/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * ESP deck firmware
 *
 * Copyright (C) 2022 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include "wifi.h"

#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/socket.h>
#include <netinet/in.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
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

static esp_routable_packet_t rxp;
static esp_routable_packet_t txp;

#define MAX_SSID_SIZE (50)
#define MAX_PASSWD_SIZE (50)

static char ssid[MAX_SSID_SIZE];
static char key[MAX_SSID_SIZE];

static const int WIFI_CONNECTED_BIT = BIT0;
static const int WIFI_SOCKET_DISCONNECTED = BIT1;
static const int WIFI_PACKET_WAIT_SEND = BIT2;
static const int WIFI_PACKET_SENDING = BIT3;
static EventGroupHandle_t s_wifi_event_group;

static const int START_UP_MAIN_TASK = BIT0;
static const int START_UP_RX_TASK = BIT1;
static const int START_UP_TX_TASK = BIT2;
static const int START_UP_CTRL_TASK = BIT3;
static EventGroupHandle_t startUpEventGroup;

#define NO_CONNECTION -1
#define WIFI_HOST_QUEUE_LENGTH (2)

static xQueueHandle wifiRxQueue;
static xQueueHandle wifiTxQueue;

/* Log printout tag */
static const char *TAG = "WIFI";

/* Socket for receiving WiFi connections */
static int serverSock = -1;
/* Accepted WiFi connection */
static int clientConnection = NO_CONNECTION;

static int serverPeerSock = -1;
static int peerConnection = NO_CONNECTION;

#define WIFI_PEER_QUEUE_LENGTH (2)
static xQueueHandle wifiPeerRxQueue;
static xQueueHandle wifiPeerTxQueue;

enum
{
  WIFI_CTRL_SET_SSID = 0x10,
  WIFI_CTRL_SET_KEY = 0x11,

  WIFI_CTRL_WIFI_CONNECT = 0x20,

  WIFI_CTRL_STATUS_WIFI_CONNECTED = 0x31,
  WIFI_CTRL_STATUS_CLIENT_CONNECTED = 0x32,

  WIFI_CTRL_PEER_CONNECT = 0x70, // [ip(4)][port(2)] BE
  WIFI_CTRL_PEER_CLOSE = 0x71,   // -
  WIFI_CTRL_PEER_STATUS = 0x72,  // ESP32->GAP8: [up(1)]
};

// Stato della connessione corrente (peer attuale)
static uint32_t currentPeerIpBE = 0;
static uint16_t currentPeerPortBE = 0;

// Helper per mandare lo STATUS al GAP8
static inline void send_peer_status(uint8_t st)
{
  cpxInitRoute(CPX_T_ESP32, CPX_T_GAP8, CPX_F_WIFI_CTRL, &txp.route);
  txp.data[0] = WIFI_CTRL_PEER_STATUS;
  txp.data[1] = st; // 0=down, 1=up, 2=busy
  txp.dataLength = 2;
  espAppSendToRouterBlocking(&txp);
}

/* WiFi event handler */
static void event_handler(void *handlerArg, esp_event_base_t eventBase, int32_t eventId, void *eventData)
{
  if (eventBase == WIFI_EVENT)
  {
    switch (eventId)
    {
    case WIFI_EVENT_STA_START:
      ESP_ERROR_CHECK(esp_wifi_connect());
      break;
    case WIFI_EVENT_STA_DISCONNECTED:
      ESP_ERROR_CHECK(esp_wifi_connect());
      xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
      ESP_LOGI(TAG, "Disconnected from access point");
      break;
    case WIFI_EVENT_AP_STACONNECTED:
    {
      wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *)eventData;
      ESP_LOGI(TAG, "station:" MACSTR " join, AID=%d",
               MAC2STR(event->mac),
               event->aid);
    }
    break;
    case WIFI_EVENT_AP_STADISCONNECTED:
    {
      wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *)eventData;
      ESP_LOGI(TAG, "station:" MACSTR "leave, AID=%d",
               MAC2STR(event->mac),
               event->aid);
    }
    break;
    default:
      // Fall through
      break;
    }
  }

  if (eventBase == IP_EVENT)
  {
    switch (eventId)
    {
    case IP_EVENT_STA_GOT_IP:
    {
      ip_event_got_ip_t *event = (ip_event_got_ip_t *)eventData;
      ESP_LOGI(TAG, "got ip: " IPSTR, IP2STR(&event->ip_info.ip));

      wifi_ap_record_t ap_info;
      ESP_ERROR_CHECK(esp_wifi_sta_get_ap_info(&ap_info));
      ESP_LOGD(TAG, "BSAP MAC is %x:%x:%x:%x:%x:%x",
               ap_info.bssid[0], ap_info.bssid[1], ap_info.bssid[2],
               ap_info.bssid[3], ap_info.bssid[4], ap_info.bssid[5]);
      ESP_LOGI(TAG, "country: %s", ap_info.country.cc);
      ESP_LOGI(TAG, "rssi: %d", ap_info.rssi);
      ESP_LOGI(TAG, "11b: %d, 11g: %d, 11n: %d, lr: %d",
               ap_info.phy_11b, ap_info.phy_11g, ap_info.phy_11n, ap_info.phy_lr);

      cpxInitRoute(CPX_T_ESP32, CPX_T_GAP8, CPX_F_WIFI_CTRL, &txp.route);
      txp.data[0] = WIFI_CTRL_STATUS_WIFI_CONNECTED;
      memcpy(&txp.data[1], &event->ip_info.ip.addr, sizeof(uint32_t));
      txp.dataLength = 1 + sizeof(uint32_t);

      // TODO: We should probably not block here...
      espAppSendToRouterBlocking(&txp);

      txp.route.destination = CPX_T_STM32;
      espAppSendToRouterBlocking(&txp);

      xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
    break;
    default:
      // Fall through
      break;
    }
  }
}

/* Initialize WiFi as AP */
static void wifi_init_softap(const char *ssid, const char *key)
{
  esp_netif_t *ap_netif = esp_netif_create_default_wifi_ap();
  assert(ap_netif);

  wifi_config_t wifi_config = {
      .ap = {
          .ssid_len = strlen(ssid),
          .max_connection = 1,
          .authmode = WIFI_AUTH_OPEN},
  };
  strncpy((char *)wifi_config.ap.ssid, ssid, strlen(ssid));
  if (strlen(key) > 0)
  {
    strncpy((char *)wifi_config.ap.password, key, strlen(key));
    wifi_config.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;
  }

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
  ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());

  ESP_LOGI(TAG, "wifi_init_softap finished");
}

static void wifi_init_sta(const char *ssid, const char *key)
{
  esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
  assert(sta_netif);

  wifi_config_t wifi_config;
  memset((void *)&wifi_config, 0, sizeof(wifi_config_t));
  strncpy((char *)wifi_config.sta.ssid, ssid, strlen(ssid));
  ESP_LOGD(TAG, "SSID is %u chars", strlen(ssid));
  strncpy((char *)wifi_config.sta.password, key, strlen(key));
  ESP_LOGD(TAG, "KEY is %u chars", strlen(key));

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
  ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());

  ESP_LOGI(TAG, "wifi_init_sta finished.");
}

static void close_peer_socket()
{
  if (peerConnection != NO_CONNECTION)
  {
    close(peerConnection);
    peerConnection = NO_CONNECTION;
    currentPeerIpBE = 0;
    currentPeerPortBE = 0;
    send_peer_status(0); // down
  }
}

static void wifi_ctrl(void *_param)
{
  xEventGroupSetBits(startUpEventGroup, START_UP_CTRL_TASK);

  while (1)
  {
    com_receive_wifi_ctrl_blocking(&rxp);

    switch (rxp.data[0])
    {
    case WIFI_CTRL_PEER_CONNECT:
    {
      // ESP_LOGI(TAG, "[CTRL] Peer connect request");

      // payload: [ip(4)][port(2)] in network order (BE)
      const uint8_t *p = &rxp.data[1];
      uint32_t ip_be;
      memcpy(&ip_be, p, 4);
      p += 4;
      uint16_t port_be;
      memcpy(&port_be, p, 2);

      uint32_t ip_ho = ntohl(ip_be);
      uint16_t port_ho = ntohs(port_be);
      char ip_str[16];
      snprintf(ip_str, sizeof(ip_str), "%u.%u.%u.%u",
               (ip_ho >> 24) & 0xFF, (ip_ho >> 16) & 0xFF,
               (ip_ho >> 8) & 0xFF, ip_ho & 0xFF);
      ESP_LOGI(TAG, "[PEER] Connect request to %s:%u", ip_str, port_ho);

      // 1) Già connesso allo stesso peer -> OK (UP)
      if (peerConnection != NO_CONNECTION &&
          currentPeerIpBE == ip_be && currentPeerPortBE == port_be)
      {
        // ESP_LOGI(TAG, "[PEER] Already connected to requested peer");
        send_peer_status(1);
        break;
      }

      // 2) Connesso ad altro peer -> BUSY quindi non chiudo
      if (peerConnection != NO_CONNECTION)
      {
        // ESP_LOGW(TAG, "[PEER] Already connected to a different peer -> busy");
        send_peer_status(2); // busy
        break;
      }

      // 3) creo socket e provo a connettermi con timeout
      int fd = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
      if (fd < 0)
      {
        // ESP_LOGE(TAG, "[PEER] socket() errno=%d", errno);
        send_peer_status(0); // down
        break;
      }

      int on = 1;
      setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));
      setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &on, sizeof(on));

      struct sockaddr_in peer = {0};
      peer.sin_family = AF_INET;
      peer.sin_addr.s_addr = ip_be; // già BE
      peer.sin_port = port_be;      // già BE

      // Non-blocking per avere timeout controllato
      int fl = fcntl(fd, F_GETFL, 0);
      fcntl(fd, F_SETFL, fl | O_NONBLOCK);

      bool ok = false;
      int rc = connect(fd, (struct sockaddr *)&peer, sizeof(peer));
      if (rc == 0)
      {
        ok = true; // connesso
      }
      else if (errno == EINPROGRESS)
      {
        // Handshake in corso: attendo fino a 2s che diventi writable (connesso) o eccezione (fallito)
        fd_set wf, ef;
        FD_ZERO(&wf);
        FD_ZERO(&ef);
        FD_SET(fd, &wf);
        FD_SET(fd, &ef);
        struct timeval tv = {.tv_sec = 2, .tv_usec = 0}; // timeout 2s
        int sel = select(fd + 1, NULL, &wf, &ef, &tv);
        if (sel > 0 && FD_ISSET(fd, &wf))
        {
          int soerr = 0;
          socklen_t slen = sizeof(soerr);
          getsockopt(fd, SOL_SOCKET, SO_ERROR, &soerr, &slen);
          ok = (soerr == 0);
        }
        else
        {
          // timeout (sel==0) o errore (sel<0) o eccezione (ef)
          ok = false;
        }
      }
      else
      {
        ok = false; // fallito
      }

      if (ok)
      {
        if (peerConnection != NO_CONNECTION)
        {
          close(fd);           // evito leak
          send_peer_status(1); // up
          break;
        }
        // rimetto blocking
        int fl2 = fcntl(fd, F_GETFL, 0);
        fcntl(fd, F_SETFL, fl2 & ~O_NONBLOCK);

        peerConnection = fd;
        currentPeerIpBE = ip_be;
        currentPeerPortBE = port_be;

        struct timeval to = {.tv_sec = 0, .tv_usec = 500000};
        setsockopt(peerConnection, SOL_SOCKET, SO_SNDTIMEO, &to, sizeof(to));
        setsockopt(peerConnection, SOL_SOCKET, SO_RCVTIMEO, &to, sizeof(to));
        int on = 1;
        setsockopt(peerConnection, SOL_SOCKET, SO_KEEPALIVE, &on, sizeof(on));

        ESP_LOGI(TAG, "[PEER] Connect established");
        send_peer_status(1); // up
      }
      else
      {
        // ESP_LOGW(TAG, "[PEER] Connect failed/timeout, errno=%d", errno);
        close(fd);
        send_peer_status(0); // down
      }
      break;
    }
    case WIFI_CTRL_PEER_CLOSE:
    {
      if (peerConnection != NO_CONNECTION)
      {
        close_peer_socket();
        ESP_LOGI(TAG, "[CTRL] Peer connection closed on request");
      }
      else
      {
        send_peer_status(0); // down
      }
      break;
    }

    case WIFI_CTRL_SET_SSID:
      ESP_LOGD("WIFI", "Should set SSID");
      memcpy(ssid, &rxp.data[1], rxp.dataLength - 1);
      ssid[rxp.dataLength - 1 + 1] = 0;
      ESP_LOGD(TAG, "SSID: %s", ssid);
      break;

    case WIFI_CTRL_SET_KEY:
      ESP_LOGD("WIFI", "Should set password");
      memcpy(key, &rxp.data[1], rxp.dataLength - 1);
      key[rxp.dataLength - 1 + 1] = 0;
      ESP_LOGD(TAG, "KEY: %s", key);
      break;

    case WIFI_CTRL_WIFI_CONNECT:
      ESP_LOGD("WIFI", "Should connect");
      if (strlen(ssid) > 0)
      {
        if (rxp.data[1] == 0)
        {
          wifi_init_sta(ssid, key);
        }
        else
        {
          if (0 < strlen(key) && strlen(key) < 8)
          {
            ESP_LOGW(TAG, "Password too short, cannot initialize AP");
          }
          else
          {
            wifi_init_softap(ssid, key);
          }
        }
      }
      else
      {
        ESP_LOGW(TAG, "No SSID set, cannot start wifi");
      }
      break;
    }
  }
}

static void close_client_socket()
{
  close(clientConnection);
  clientConnection = NO_CONNECTION;
  xEventGroupSetBits(s_wifi_event_group, WIFI_SOCKET_DISCONNECTED);
}

void wifi_bind_socket()
{
  char addr_str[128];
  int addr_family;
  int ip_protocol;
  struct sockaddr_in destAddr;
  destAddr.sin_addr.s_addr = htonl(INADDR_ANY);
  destAddr.sin_family = AF_INET;
  destAddr.sin_port = htons(5000);
  addr_family = AF_INET;
  ip_protocol = IPPROTO_IP;
  inet_ntoa_r(destAddr.sin_addr, addr_str, sizeof(addr_str) - 1);
  serverSock = socket(addr_family, SOCK_STREAM, ip_protocol);
  if (serverSock < 0)
  {
    ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
  }
  ESP_LOGD(TAG, "Socket created");

  int err = bind(serverSock, (struct sockaddr *)&destAddr, sizeof(destAddr));
  if (err != 0)
  {
    ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
  }
  ESP_LOGD(TAG, "Socket binded");

  err = listen(serverSock, 1);
  if (err != 0)
  {
    ESP_LOGE(TAG, "Error occured during listen: errno %d", errno);
  }
  ESP_LOGD(TAG, "Socket listening");
}

static void wifi_peer_bind_socket()
{
  struct sockaddr_in destAddr = {0};
  destAddr.sin_addr.s_addr = htonl(INADDR_ANY);
  destAddr.sin_family = AF_INET;
  destAddr.sin_port = htons(WIFI_PEER_TCP_PORT);

  serverPeerSock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
  if (serverPeerSock < 0)
  {
    // ESP_LOGE(TAG, "[PEER] Unable to create socket: errno %d", errno);
    return;
  }
  int on = 1;
  setsockopt(serverPeerSock, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));

  if (bind(serverPeerSock, (struct sockaddr *)&destAddr, sizeof(destAddr)) != 0)
  {
    // ESP_LOGE(TAG, "[PEER] bind: errno %d", errno);
    close(serverPeerSock);
    serverPeerSock = -1;
    return;
  }
  if (listen(serverPeerSock, 1) != 0)
  {
    // ESP_LOGE(TAG, "[PEER] listen: errno %d", errno);
    close(serverPeerSock);
    serverPeerSock = -1;
    return;
  }
  // ESP_LOGI(TAG, "[PEER] Listening on :%d", WIFI_PEER_TCP_PORT);
}

void wifi_wait_for_socket_connected()
{
  // ESP_LOGI(TAG, "Waiting for connection");
  struct sockaddr sourceAddr;
  uint addrLen = sizeof(sourceAddr);
  clientConnection = accept(serverSock, (struct sockaddr *)&sourceAddr, &addrLen);
  if (clientConnection < 0)
  {
    ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
  }
  else
  {
    ESP_LOGI(TAG, "Connection accepted");
  }
}

static void wifi_peer_wait_for_socket_connected()
{
  ESP_LOGI(TAG, "[PEER] Waiting for connection");
  struct sockaddr_in sourceAddr;
  socklen_t addrLen = sizeof(sourceAddr);

  int fd = accept(serverPeerSock, (struct sockaddr *)&sourceAddr, &addrLen);

  if (fd < 0)
  {
    ESP_LOGE(TAG, "[PEER] Unable to accept connection: errno %d", errno);
    return;
  }

  peerConnection = fd;                          // accetto la connessione
  currentPeerIpBE = sourceAddr.sin_addr.s_addr; // già BE
  currentPeerPortBE = sourceAddr.sin_port;      // già BE
  // ESP_LOGI(TAG, "[PEER] Connection accepted");

  struct timeval to = {.tv_sec = 0, .tv_usec = 500000};
  setsockopt(peerConnection, SOL_SOCKET, SO_SNDTIMEO, &to, sizeof(to));
  setsockopt(peerConnection, SOL_SOCKET, SO_RCVTIMEO, &to, sizeof(to));
  int on = 1;
  setsockopt(peerConnection, SOL_SOCKET, SO_KEEPALIVE, &on, sizeof(on));

  uint32_t ip_ho = ntohl(currentPeerIpBE);
  uint16_t port_ho = ntohs(currentPeerPortBE);
  ESP_LOGI(TAG, "[PEER] connected from %u.%u.%u.%u:%u",
           (ip_ho >> 24) & 0xFF, (ip_ho >> 16) & 0xFF,
           (ip_ho >> 8) & 0xFF, ip_ho & 0xFF, port_ho);

  // notifica UP al GAP8
  send_peer_status(1);
}

void wifi_wait_for_disconnect()
{
  xEventGroupWaitBits(s_wifi_event_group, WIFI_SOCKET_DISCONNECTED, pdTRUE, pdFALSE, portMAX_DELAY);
}

static void wifi_task(void *pvParameters)
{

  s_wifi_event_group = xEventGroupCreate();

  esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, event_handler, NULL, NULL);
  esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, event_handler, NULL, NULL);

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  uint8_t mac[6];
  ESP_ERROR_CHECK(esp_wifi_get_mac(ESP_IF_WIFI_AP, mac));
  ESP_LOGD(TAG, "AP MAC is %x:%x:%x:%x:%x:%x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  ESP_ERROR_CHECK(esp_wifi_get_mac(ESP_IF_WIFI_STA, mac));
  ESP_LOGD(TAG, "STA MAC is %x:%x:%x:%x:%x:%x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  wifi_bind_socket();
  wifi_peer_bind_socket();

  xEventGroupSetBits(startUpEventGroup, START_UP_MAIN_TASK);

  // vTaskDelete(NULL);
  while (1)
  {
    // blink_period_ms = 500;
    wifi_wait_for_socket_connected();
    ESP_LOGI(TAG, "Client connected");

    // blink_period_ms = 100;

    // Not thread safe!
    cpxInitRoute(CPX_T_ESP32, CPX_T_GAP8, CPX_F_WIFI_CTRL, &txp.route);
    txp.data[0] = WIFI_CTRL_STATUS_CLIENT_CONNECTED;
    txp.data[1] = 1; // connected
    txp.dataLength = 2;
    espAppSendToRouterBlocking(&txp);

    txp.route.destination = CPX_T_STM32;
    espAppSendToRouterBlocking(&txp);

    // Probably not the best, should be handled in some other way?
    wifi_wait_for_disconnect();
    ESP_LOGI(TAG, "Client disconnected");

    // Not thread safe!
    cpxInitRoute(CPX_T_ESP32, CPX_T_GAP8, CPX_F_WIFI_CTRL, &txp.route);
    txp.data[0] = WIFI_CTRL_STATUS_CLIENT_CONNECTED;
    txp.data[1] = 0; // disconnected
    txp.dataLength = 2;
    espAppSendToRouterBlocking(&txp);

    txp.route.destination = CPX_T_STM32;
    espAppSendToRouterBlocking(&txp);
  }
}

static void wifi_peer_accept_task(void *pvParameters)
{
  for (;;)
  {
    if (peerConnection == NO_CONNECTION && serverPeerSock >= 0)
    {
      // blocca finché arriva un peer
      wifi_peer_wait_for_socket_connected();
      /**
       * if (peerConnection != NO_CONNECTION)
      {
        // notifica UP al GAP8 (riusa il tuo canale CTRL)
        send_peer_status(1); // up
      }
       */
    }
    else
    {
      vTaskDelay(pdMS_TO_TICKS(50));
    }
  }
}

void wifi_led_task(void *pvParameters)
{
  int ledstate = 0;
  while (1)
  {
    if (peerConnection == NO_CONNECTION)
    {
      gpio_set_level(BLINK_GPIO, !ledstate);
      ledstate = !ledstate;
      vTaskDelay(pdMS_TO_TICKS(500));
    }
    else
    {
      gpio_set_level(BLINK_GPIO, 1);
      vTaskDelay(pdMS_TO_TICKS(200));
      EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_PACKET_SENDING | WIFI_PACKET_WAIT_SEND, pdFALSE, pdFALSE, portMAX_DELAY);
      if (bits & WIFI_PACKET_SENDING)
      {
        gpio_set_level(BLINK_GPIO, 1);
        xEventGroupClearBits(s_wifi_event_group, WIFI_PACKET_SENDING);
      }
      if (bits & WIFI_PACKET_WAIT_SEND)
      {
        gpio_set_level(BLINK_GPIO, 0);
        xEventGroupClearBits(s_wifi_event_group, WIFI_PACKET_WAIT_SEND);
      }
    }
  }
}

void wifi_send_packet(const char *buffer, size_t size)
{
  if (clientConnection != NO_CONNECTION)
  {
    ESP_LOGD(TAG, "Sending WiFi packet of size %u", size);
    xEventGroupSetBits(s_wifi_event_group, WIFI_PACKET_SENDING);
    int err = send(clientConnection, buffer, size, 0);
    if (err < 0)
    {
      ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
      close_client_socket();
    }
    xEventGroupSetBits(s_wifi_event_group, WIFI_PACKET_WAIT_SEND);
  }
}

static void wifi_peer_send_packet(const char *buffer, size_t size)
{
  if (peerConnection == NO_CONNECTION)
    return;
  int sent = send(peerConnection, buffer, size, 0);
  if (sent < 0)
  {
    ESP_LOGE(TAG, "[PEER] send error/timeout -> closing. errno=%d", errno);
    close_peer_socket();
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

static void wifi_peer_sending_task(void *pvParameters)
{
  static WifiTransportPacket_t txp_wifi;
  static CPXRoutablePacket_t qPacket;
  while (1)
  {
    xEventGroupSetBits(startUpEventGroup, START_UP_TX_TASK);
    xQueueReceive(wifiPeerTxQueue, &qPacket, portMAX_DELAY);
    txp_wifi.payloadLength = qPacket.dataLength + CPX_ROUTING_PACKED_SIZE;
    cpxRouteToPacked(&qPacket.route, &txp_wifi.routablePayload.route);
    memcpy(txp_wifi.routablePayload.data, qPacket.data, qPacket.dataLength);
    wifi_peer_send_packet((const char *)&txp_wifi, txp_wifi.payloadLength + 2);
  }
}

static void wifi_receiving_task(void *pvParameters)
{
  static WifiTransportPacket_t rxp_wifi;
  int len;

  xEventGroupSetBits(startUpEventGroup, START_UP_RX_TASK);
  while (1)
  {
    len = recv(clientConnection, &rxp_wifi, 2, 0);
    if (len > 0)
    {
      ESP_LOGD(TAG, "Wire data length %i", rxp_wifi.payloadLength);
      int totalRxLen = 0;
      do
      {
        len = recv(clientConnection, &rxp_wifi.payload[totalRxLen], rxp_wifi.payloadLength - totalRxLen, 0);
        ESP_LOGD(TAG, "Read %i bytes", len);
        totalRxLen += len;
      } while (totalRxLen < rxp_wifi.payloadLength);
      ESP_LOG_BUFFER_HEX_LEVEL(TAG, &rxp_wifi, 10, ESP_LOG_DEBUG);
      xQueueSend(wifiRxQueue, &rxp_wifi, portMAX_DELAY);
    }
    else if (len == 0)
    {
      close_client_socket(); // Reading 0 bytes most often means the client has disconnected.
    }
    else
    {
      vTaskDelay(10);
    }
  }
}

static void wifi_peer_receiving_task(void *pvParameters)
{
  xEventGroupSetBits(startUpEventGroup, START_UP_RX_TASK);
  static WifiTransportPacket_t rxp_wifi;

  for (;;)
  {
    if (peerConnection == NO_CONNECTION)
    {
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }

    // === 1) leggo 2 byte di header (payloadLength) ===
    uint8_t hdr[2];
    int need = 2, got = 0;
    while (got < need)
    {
      int r = recv(peerConnection, hdr + got, need - got, 0);
      if (r == 0)
      { // peer ha chiuso
        close_peer_socket();
        goto next_iter;
      }
      if (r < 0)
      {
        // timeout/errore
        vTaskDelay(pdMS_TO_TICKS(10));
        goto next_iter;
      }
      got += r;
    }

    // ricostruisco
    uint16_t payloadLen = 0;
    memcpy(&payloadLen, hdr, 2);

    if (payloadLen == 0 || payloadLen > sizeof(rxp_wifi.payload))
    {
      ESP_LOGW(TAG, "[PEER] invalid payloadLength=%u -> closing", payloadLen);
      close_peer_socket();
      goto next_iter;
    }

    rxp_wifi.payloadLength = payloadLen; // opzionale: per coerenza interna

    // === 2) leggo payload
    int total = 0;
    while (total < payloadLen)
    {
      int r = recv(peerConnection, &rxp_wifi.payload[total], payloadLen - total, 0);
      if (r == 0)
      { // peer ha chiuso durante il payload
        close_peer_socket();
        goto next_iter;
      }
      if (r < 0)
      {
        // timeout/errore temporaneo: meglio chiudere
        ESP_LOGW(TAG, "[PEER] recv payload timeout/err -> closing");
        close_peer_socket();
        goto next_iter;
      }
      total += r;
    }

    // === 3) Consegnalo alla coda del router ===
    xQueueSend(wifiPeerRxQueue, &rxp_wifi, portMAX_DELAY);

  next_iter:; // nulla
  }
}

void wifi_transport_send(const CPXRoutablePacket_t *packet)
{
  assert(packet->dataLength <= WIFI_TRANSPORT_MTU - CPX_ROUTING_PACKED_SIZE);
  xQueueSend(wifiTxQueue, packet, portMAX_DELAY);
}

/* ---- Transport PEER (4242) esposto al router ---- */
void wifi_peer_transport_send(const CPXRoutablePacket_t *packet)
{
  assert(packet->dataLength <= WIFI_TRANSPORT_MTU - CPX_ROUTING_PACKED_SIZE);
  xQueueSend(wifiPeerTxQueue, packet, portMAX_DELAY);
}

void wifi_transport_receive(CPXRoutablePacket_t *packet)
{
  // Not reentrant safe. Assuming only one task is popping the queue
  static WifiTransportPacket_t qPacket;
  xQueueReceive(wifiRxQueue, &qPacket, portMAX_DELAY);

  packet->dataLength = qPacket.payloadLength - CPX_ROUTING_PACKED_SIZE;

  cpxPackedToRoute(&qPacket.routablePayload.route, &packet->route);

  memcpy(packet->data, qPacket.routablePayload.data, packet->dataLength);
}

void wifi_peer_transport_receive(CPXRoutablePacket_t *packet)
{
  static WifiTransportPacket_t qPacket;
  xQueueReceive(wifiPeerRxQueue, &qPacket, portMAX_DELAY);
  packet->dataLength = qPacket.payloadLength - CPX_ROUTING_PACKED_SIZE;
  cpxPackedToRoute(&qPacket.routablePayload.route, &packet->route);
  memcpy(packet->data, qPacket.routablePayload.data, packet->dataLength);
}

void wifi_init()
{
  esp_netif_init();

  s_wifi_event_group = xEventGroupCreate();

  wifiRxQueue = xQueueCreate(WIFI_HOST_QUEUE_LENGTH, sizeof(WifiTransportPacket_t)); // per ora non commento
  wifiTxQueue = xQueueCreate(WIFI_HOST_QUEUE_LENGTH, sizeof(CPXRoutablePacket_t));   // per ora non commento
  wifiPeerRxQueue = xQueueCreate(WIFI_PEER_QUEUE_LENGTH, sizeof(WifiTransportPacket_t));
  wifiPeerTxQueue = xQueueCreate(WIFI_PEER_QUEUE_LENGTH, sizeof(CPXRoutablePacket_t));

  startUpEventGroup = xEventGroupCreate();
  xEventGroupClearBits(startUpEventGroup, START_UP_MAIN_TASK | START_UP_RX_TASK | START_UP_TX_TASK | START_UP_CTRL_TASK);
  xTaskCreate(wifi_task, "WiFi TASK", 5000, NULL, 1, NULL);
  xTaskCreate(wifi_sending_task, "WiFi TX", 5000, NULL, 1, NULL);   // per ora non commento
  xTaskCreate(wifi_receiving_task, "WiFi RX", 5000, NULL, 1, NULL); // per ora non commento
  xTaskCreate(wifi_peer_accept_task, "WiFi PEER ACCEPT", 4096, NULL, 1, NULL);
  xTaskCreate(wifi_peer_sending_task, "WiFi PEER TX", 5000, NULL, 1, NULL);
  xTaskCreate(wifi_peer_receiving_task, "WiFi PEER RX", 5000, NULL, 1, NULL);
  xTaskCreate(wifi_led_task, "WiFi LED", 5000, NULL, 1, NULL);
  ESP_LOGI(TAG, "Waiting for main, RX and TX tasks to start");
  xEventGroupWaitBits(startUpEventGroup,
                      START_UP_MAIN_TASK | START_UP_RX_TASK | START_UP_TX_TASK,
                      pdTRUE, // Clear bits before returning
                      pdTRUE, // Wait for all bits
                      portMAX_DELAY);

  xTaskCreate(wifi_ctrl, "WiFi CTRL", 5000, NULL, 1, NULL);
  ESP_LOGI(TAG, "Waiting for CTRL task to start");
  xEventGroupWaitBits(startUpEventGroup,
                      START_UP_CTRL_TASK,
                      pdTRUE, // Clear bits before returning
                      pdTRUE, // Wait for all bits
                      portMAX_DELAY);

  ESP_LOGI("WIFI", "Wifi initialized");
}
