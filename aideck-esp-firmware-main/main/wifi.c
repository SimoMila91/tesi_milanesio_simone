#include "wifi.h"
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/sockets.h"
#include <lwip/netdb.h>
#include "esp_netif.h"
#include "com.h"

#define BLINK_GPIO 4
#define MCAST_GRP  "239.255.42.42"
#define MCAST_PORT 4242

static const char *TAG = "WIFI";

// global vars 
static EventGroupHandle_t s_wifi_event_group;
static EventGroupHandle_t startUpEventGroup;
static xQueueHandle wifiRxQueue;
static xQueueHandle wifiTxQueue;
static xQueueHandle wifiPeerRxQueue;
static xQueueHandle wifiPeerTxQueue;

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_PACKET_WAIT_SEND BIT1
#define WIFI_PACKET_SENDING BIT2
#define START_UP_MAIN_TASK BIT0
#define START_UP_RX_TASK   BIT1
#define START_UP_TX_TASK   BIT2
#define START_UP_CTRL_TASK BIT3

static int clientConnection = -1;
static int mcastSock = -1;
static struct sockaddr_in mcastAddr;
static uint32_t localIpBE = 0;

static char ssid[50];
static char key[50];

enum {
  WIFI_CTRL_SET_SSID = 0x10,
  WIFI_CTRL_SET_KEY  = 0x11,
  WIFI_CTRL_WIFI_CONNECT = 0x20,
  WIFI_CTRL_STATUS_WIFI_CONNECTED = 0x31,
  WIFI_CTRL_STATUS_CLIENT_CONNECTED = 0x32,
  WIFI_CTRL_PEER_STATUS = 0x72
};

 // helper per mandare status al gap8 (peer)
static inline void send_peer_status(uint8_t st)
{
  esp_routable_packet_t tx;
  cpxInitRoute(CPX_T_ESP32, CPX_T_GAP8, CPX_F_WIFI_CTRL, &tx.route);
  tx.data[0] = WIFI_CTRL_PEER_STATUS;
  tx.data[1] = st;
  tx.dataLength = 2;
  espAppSendToRouterBlocking(&tx);
}

// join/leave multicast
static bool wifi_mcast_open_and_join(uint32_t ip_be)
{
  mcastSock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (mcastSock < 0) {
    ESP_LOGE(TAG, "[MCAST] socket errno=%d", errno);
    return false;
  }

  int reuse = 1;
  setsockopt(mcastSock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

  struct sockaddr_in bindAddr = {0};
  bindAddr.sin_family = AF_INET;
  bindAddr.sin_port = htons(MCAST_PORT);
  bindAddr.sin_addr.s_addr = htonl(INADDR_ANY);
  if (bind(mcastSock, (struct sockaddr *)&bindAddr, sizeof(bindAddr)) < 0) {
    ESP_LOGE(TAG, "[MCAST] bind errno=%d", errno);
    close(mcastSock);
    mcastSock = -1;
    return false;
  }

  struct ip_mreq imr = {0};
  imr.imr_multiaddr.s_addr = inet_addr(MCAST_GRP);
  imr.imr_interface.s_addr = ip_be;
  if (setsockopt(mcastSock, IPPROTO_IP, IP_ADD_MEMBERSHIP, &imr, sizeof(imr)) < 0) {
    ESP_LOGE(TAG, "[MCAST] IP_ADD_MEMBERSHIP errno=%d", errno);
    close(mcastSock);
    mcastSock = -1;
    return false;
  }

  uint8_t ttl = 1;
  setsockopt(mcastSock, IPPROTO_IP, IP_MULTICAST_TTL, &ttl, sizeof(ttl));
  uint8_t loop = 0;
  setsockopt(mcastSock, IPPROTO_IP, IP_MULTICAST_LOOP, &loop, sizeof(loop));

  memset(&mcastAddr, 0, sizeof(mcastAddr));
  mcastAddr.sin_family = AF_INET;
  mcastAddr.sin_port = htons(MCAST_PORT);
  mcastAddr.sin_addr.s_addr = inet_addr(MCAST_GRP);

  ESP_LOGI(TAG, "[MCAST] joined %s:%d", MCAST_GRP, MCAST_PORT);
  send_peer_status(1);
  return true;
}

static void wifi_mcast_leave_and_close(void)
{
  if (mcastSock >= 0) {
    struct ip_mreq imr = {0};
    imr.imr_multiaddr.s_addr = inet_addr(MCAST_GRP);
    imr.imr_interface.s_addr = localIpBE;
    setsockopt(mcastSock, IPPROTO_IP, IP_DROP_MEMBERSHIP, &imr, sizeof(imr));
    close(mcastSock);
    mcastSock = -1;
    send_peer_status(0);
    ESP_LOGI(TAG, "[MCAST] left/closed");
  }
}

// wifi event handler
static void event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
  if (base == WIFI_EVENT) {
    if (id == WIFI_EVENT_STA_START)
      esp_wifi_connect();
    else if (id == WIFI_EVENT_STA_DISCONNECTED) {
      wifi_mcast_leave_and_close();
      esp_wifi_connect();
      xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
  }
  if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)data;
    localIpBE = event->ip_info.ip.addr;
    ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
    esp_routable_packet_t txp; 
    cpxInitRoute(CPX_T_ESP32, CPX_T_GAP8, CPX_F_WIFI_CTRL, &txp.route);
    txp.data[0] = WIFI_CTRL_STATUS_WIFI_CONNECTED;
    memcpy(&txp.data[1], &event->ip_info.ip.addr, sizeof(uint32_t));
    txp.dataLength = 1 + sizeof(uint32_t);
    espAppSendToRouterBlocking(&txp);
    wifi_mcast_open_and_join(localIpBE);
    xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
  }
}

// init sta/ap
static void wifi_init_sta(const char *ssid, const char *key)
{
  esp_netif_create_default_wifi_sta();
  wifi_config_t cfg = {0};
  strncpy((char *)cfg.sta.ssid, ssid, sizeof(cfg.sta.ssid));
  strncpy((char *)cfg.sta.password, key, sizeof(cfg.sta.password));
  esp_wifi_set_mode(WIFI_MODE_STA);
  esp_wifi_set_config(ESP_IF_WIFI_STA, &cfg);
  esp_wifi_start();
}

// ctrl handler 
static void wifi_ctrl(void *p)
{
  xEventGroupSetBits(startUpEventGroup, START_UP_CTRL_TASK);
  while (1) {
    esp_routable_packet_t rxp;
    com_receive_wifi_ctrl_blocking(&rxp);
    switch (rxp.data[0]) {
      case WIFI_CTRL_SET_SSID:
        memcpy(ssid, &rxp.data[1], rxp.dataLength - 1);
        ssid[rxp.dataLength] = 0;
        break;
      case WIFI_CTRL_SET_KEY:
        memcpy(key, &rxp.data[1], rxp.dataLength - 1);
        key[rxp.dataLength] = 0;
        break;
      case WIFI_CTRL_WIFI_CONNECT:
        wifi_init_sta(ssid, key);
        break;
    }
  }
}

// muticast tx-rx
static void wifi_mcast_send_packet(const char *buf, size_t len)
{
  if (mcastSock < 0) return;
  sendto(mcastSock, buf, len, 0, (struct sockaddr *)&mcastAddr, sizeof(mcastAddr));
}

static void wifi_mcast_sending_task(void *p)
{
  static WifiTransportPacket_t txp_wifi;
  static CPXRoutablePacket_t qPacket;
  xEventGroupSetBits(startUpEventGroup, START_UP_TX_TASK);
  while (1) {
    xQueueReceive(wifiPeerTxQueue, &qPacket, portMAX_DELAY);
    txp_wifi.payloadLength = qPacket.dataLength + CPX_ROUTING_PACKED_SIZE;
    cpxRouteToPacked(&qPacket.route, &txp_wifi.routablePayload.route);
    memcpy(txp_wifi.routablePayload.data, qPacket.data, qPacket.dataLength);
    wifi_mcast_send_packet((const char *)&txp_wifi, txp_wifi.payloadLength + 2);
  }
}

static void wifi_mcast_receiving_task(void *p)
{
  static WifiTransportPacket_t rxp_wifi;
  xEventGroupSetBits(startUpEventGroup, START_UP_RX_TASK);
  for (;;) {
    if (mcastSock < 0) { vTaskDelay(pdMS_TO_TICKS(100)); continue; }
    uint8_t buf[sizeof(WifiTransportPacket_t)];
    int r = recvfrom(mcastSock, buf, sizeof(buf), 0, NULL, NULL);
    if (r < 2) continue;
    uint16_t payloadLen;
    memcpy(&payloadLen, buf, 2);
    if (payloadLen == 0 || payloadLen > sizeof(rxp_wifi.payload)) continue;
    rxp_wifi.payloadLength = payloadLen;
    memcpy(rxp_wifi.payload, buf + 2, payloadLen);
    xQueueSend(wifiPeerRxQueue, &rxp_wifi, portMAX_DELAY);
  }
}

/* ===== LED task ===== */
void wifi_led_task(void *p)
{
  int led = 0;
  while (1) {
    if (mcastSock < 0) {
      gpio_set_level(BLINK_GPIO, led ^= 1);
      vTaskDelay(pdMS_TO_TICKS(500));
    } else {
      gpio_set_level(BLINK_GPIO, 1);
      vTaskDelay(pdMS_TO_TICKS(200));
    }
  }
}

#define WIFI_HOST_QUEUE_LENGTH 2

static void close_client_socket()
{
  close(clientConnection);
  clientConnection = -1;
  xEventGroupSetBits(s_wifi_event_group, WIFI_PACKET_WAIT_SEND);
}

static void wifi_bind_socket()
{
  struct sockaddr_in destAddr = {0};
  destAddr.sin_addr.s_addr = htonl(INADDR_ANY);
  destAddr.sin_family = AF_INET;
  destAddr.sin_port = htons(5000);

  int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
  if (sock < 0) {
    ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
    return;
  }

  int opt = 1;
  setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

  if (bind(sock, (struct sockaddr *)&destAddr, sizeof(destAddr)) != 0) {
    ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
    close(sock);
    return;
  }
  if (listen(sock, 1) != 0) {
    ESP_LOGE(TAG, "Error during listen: errno %d", errno);
    close(sock);
    return;
  }
  ESP_LOGI(TAG, "[HOST] TCP server listening on port 5000");

  clientConnection = accept(sock, NULL, NULL);
  if (clientConnection < 0) {
    ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
    close(sock);
    return;
  }
  ESP_LOGI(TAG, "[HOST] Client connected");
  close(sock); 

void wifi_send_packet(const char *buffer, size_t size)
{
  if (clientConnection >= 0) {
    int err = send(clientConnection, buffer, size, 0);
    if (err < 0) {
      ESP_LOGE(TAG, "Error occurred during send: errno %d", errno);
      close_client_socket();
    }
  }
}

static void wifi_receiving_task(void *pvParameters)
{
  static WifiTransportPacket_t rxp_wifi;
  xEventGroupSetBits(startUpEventGroup, START_UP_RX_TASK);
  while (1)
  {
    if (clientConnection < 0) {
      wifi_bind_socket();
      continue;
    }

    int len = recv(clientConnection, &rxp_wifi, 2, 0);
    if (len <= 0) {
      ESP_LOGW(TAG, "[HOST] Client disconnected");
      close_client_socket();
      continue;
    }

    int total = 0;
    while (total < rxp_wifi.payloadLength) {
      int r = recv(clientConnection, &rxp_wifi.payload[total], rxp_wifi.payloadLength - total, 0);
      if (r <= 0) { close_client_socket(); break; }
      total += r;
    }

    xQueueSend(wifiRxQueue, &rxp_wifi, portMAX_DELAY);
  }
}

static void wifi_sending_task(void *pvParameters)
{
  static WifiTransportPacket_t txp_wifi;
  static CPXRoutablePacket_t qPacket;
  xEventGroupSetBits(startUpEventGroup, START_UP_TX_TASK);
  while (1)
  {
    xQueueReceive(wifiTxQueue, &qPacket, portMAX_DELAY);
    txp_wifi.payloadLength = qPacket.dataLength + CPX_ROUTING_PACKED_SIZE;
    cpxRouteToPacked(&qPacket.route, &txp_wifi.routablePayload.route);
    memcpy(txp_wifi.routablePayload.data, qPacket.data, qPacket.dataLength);
    wifi_send_packet((const char *)&txp_wifi, txp_wifi.payloadLength + 2);
  }
}


void wifi_init()
{
  esp_netif_init();
  s_wifi_event_group = xEventGroupCreate();
  wifiRxQueue = xQueueCreate(2, sizeof(WifiTransportPacket_t));
  wifiTxQueue = xQueueCreate(2, sizeof(CPXRoutablePacket_t));
  wifiPeerRxQueue = xQueueCreate(2, sizeof(WifiTransportPacket_t));
  wifiPeerTxQueue = xQueueCreate(2, sizeof(CPXRoutablePacket_t));

  startUpEventGroup = xEventGroupCreate();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  esp_wifi_init(&cfg);
  esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, event_handler, NULL, NULL);
  esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, event_handler, NULL, NULL);

  xTaskCreate(wifi_ctrl, "WiFi CTRL", 4096, NULL, 1, NULL);
  xTaskCreate(wifi_mcast_sending_task, "WiFi MCAST TX", 4096, NULL, 1, NULL);
  xTaskCreate(wifi_mcast_receiving_task, "WiFi MCAST RX", 4096, NULL, 1, NULL);
  xTaskCreate(wifi_led_task, "WiFi LED", 2048, NULL, 1, NULL);
  xTaskCreate(wifi_receiving_task, "WiFi RX", 4096, NULL, 1, NULL);
xTaskCreate(wifi_sending_task,   "WiFi TX", 4096, NULL, 1, NULL);


  ESP_LOGI(TAG, "WiFi init complete (UDP multicast mode)");
}


// invio pacchetto tcp (host) - porta 5000
void wifi_transport_send(const CPXRoutablePacket_t *packet)
{
    assert(packet->dataLength <= WIFI_TRANSPORT_MTU - CPX_ROUTING_PACKED_SIZE);
    xQueueSend(wifiTxQueue, packet, portMAX_DELAY);
}

// ricevo pacchetto tcp (host) - porta 5000
void wifi_transport_receive(CPXRoutablePacket_t *packet)
{
    static WifiTransportPacket_t qPacket;
    xQueueReceive(wifiRxQueue, &qPacket, portMAX_DELAY);

    packet->dataLength = qPacket.payloadLength - CPX_ROUTING_PACKED_SIZE;
    cpxPackedToRoute(&qPacket.routablePayload.route, &packet->route);
    memcpy(packet->data, qPacket.routablePayload.data, packet->dataLength);
}

// invio pacchetto al canale udp multicast 
void wifi_peer_transport_send(const CPXRoutablePacket_t *packet)
{
    assert(packet->dataLength <= WIFI_TRANSPORT_MTU - CPX_ROUTING_PACKED_SIZE);
    xQueueSend(wifiPeerTxQueue, packet, portMAX_DELAY);
}

// ricevo pacchetto dal canale multicast 
void wifi_peer_transport_receive(CPXRoutablePacket_t *packet)
{
    static WifiTransportPacket_t qPacket;
    xQueueReceive(wifiPeerRxQueue, &qPacket, portMAX_DELAY);

    packet->dataLength = qPacket.payloadLength - CPX_ROUTING_PACKED_SIZE;
    cpxPackedToRoute(&qPacket.routablePayload.route, &packet->route);
    memcpy(packet->data, qPacket.routablePayload.data, packet->dataLength);
}

