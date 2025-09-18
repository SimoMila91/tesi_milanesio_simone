#include "router.h"

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_log.h"

#include "cpx.h"
#include "spi_transport.h"
#include "uart_transport.h"
#include "esp_transport.h"
#include "wifi.h"

typedef struct
{
  CPXRoutablePacket_t txp;
} RouteContext_t;

// static RouteContext_t wifi_task_context;
// static CPXRoutablePacket_t wifiRxBuf;

static RouteContext_t gap8_task_context;
static CPXRoutablePacket_t spiRxBuf;

static RouteContext_t cf_task_context;
static CPXRoutablePacket_t uartRxBuf;

static RouteContext_t esp_task_context;
static CPXRoutablePacket_t espRxBuf;

static RouteContext_t wifi_peer_task_context; // <— NUOVO
static CPXRoutablePacket_t wifiPeerRxBuf;     // <— NUOVO

typedef void (*Receiver_t)(CPXRoutablePacket_t *packet);
typedef void (*Sender_t)(const CPXRoutablePacket_t *packet);

static const int START_UP_GAP8_ROUTER_RUNNING = BIT0;
static const int START_UP_CF_ROUTER_RUNNING = BIT1;
static const int START_UP_ESP_ROUTER_RUNNING = BIT2;
// static const int START_UP_WIFI_ROUTER_RUNNING = BIT3;
static const int START_UP_WIFI_PEER_ROUTER_RUNNING = BIT4; // <— NUOVO
static EventGroupHandle_t startUpEventGroup;
#define TAG "ROUTER"

static void splitAndSend(const CPXRoutablePacket_t *rxp, RouteContext_t *context, Sender_t sender, const uint16_t mtu)
{
  CPXRoutablePacket_t *txp = &context->txp;

  txp->route = rxp->route;

  uint16_t remainingToSend = rxp->dataLength;
  const uint8_t *startOfDataToSend = rxp->data;
  while (remainingToSend > 0)
  {
    uint16_t toSend = remainingToSend;
    bool lastPacket = rxp->route.lastPacket;
    if (toSend > mtu)
    {
      toSend = mtu;
      lastPacket = false;
    }

    memcpy(txp->data, startOfDataToSend, toSend);
    txp->dataLength = toSend;
    txp->route.lastPacket = lastPacket;
    sender(txp);

    remainingToSend -= toSend;
    startOfDataToSend += toSend;
  }
}

// Receiver per il peer: forza consegna al GAP8 locale
static void wifi_peer_receive_and_fix(CPXRoutablePacket_t *packet)
{
  wifi_peer_transport_receive(packet);
  packet->route.destination = CPX_T_GAP8;
  packet->route.source = CPX_T_ESP32;
}

static void route(Receiver_t receive, CPXRoutablePacket_t *rxp, RouteContext_t *context, const char *routerName)
{
  // static uint32_t lastLogTime = 0;
  while (1)
  {
    receive(rxp);
    // uint32_t now = xTaskGetTickCount();

    if (CPX_VERSION == rxp->route.version)
    {
      const CPXTarget_t source = rxp->route.source;
      const CPXTarget_t destination = rxp->route.destination;
      // const uint16_t cpxDataLength = rxp->dataLength;

      /**
       *
      if (now - lastLogTime > pdMS_TO_TICKS(10000))
      {
        ESP_LOGI("ROUTER", "%s: Received packet from [0x%02X] to [0x%02X], length [%u]", routerName, source, destination, cpxDataLength);
        lastLogTime = now;
      }
       */

      switch (destination)
      {
      case CPX_T_GAP8:
        splitAndSend(rxp, context, spi_transport_send, SPI_TRANSPORT_MTU - CPX_ROUTING_PACKED_SIZE);
        break;
      case CPX_T_STM32:
        splitAndSend(rxp, context, uart_transport_send, UART_TRANSPORT_MTU - CPX_ROUTING_PACKED_SIZE);
        break;
      case CPX_T_ESP32:
        splitAndSend(rxp, context, espTransportSend, ESP_TRANSPORT_MTU - CPX_ROUTING_PACKED_SIZE);
        break;
        /**
         *  case CPX_T_WIFI_HOST:
           splitAndSend(rxp, context, wifi_transport_send, WIFI_TRANSPORT_MTU - CPX_ROUTING_PACKED_SIZE);
           break;
         */
      case CPX_T_WIFI_PEER: // <— NUOVO: TX sul peer
        splitAndSend(rxp, context, wifi_peer_transport_send, WIFI_TRANSPORT_MTU - CPX_ROUTING_PACKED_SIZE);
        break;
      default:
        ESP_LOGW("ROUTER", "Cannot route from %s [0x%02X] to [0x%02X]", routerName, source, destination);
      }
    }
    else
    {
      ESP_LOGW("ROUTER", "Packet is invalid");
    }
  }
}

static void router_from_gap8(void *_param)
{
  xEventGroupSetBits(startUpEventGroup, START_UP_GAP8_ROUTER_RUNNING);
  route(spi_transport_receive, &spiRxBuf, &gap8_task_context, "GAP8");
}

static void router_from_crazyflie(void *_param)
{
  xEventGroupSetBits(startUpEventGroup, START_UP_CF_ROUTER_RUNNING);
  route(uart_transport_receive, &uartRxBuf, &cf_task_context, "STM32");
}

static void router_from_esp32(void *_param)
{
  xEventGroupSetBits(startUpEventGroup, START_UP_ESP_ROUTER_RUNNING);
  route(espTransportReceive, &espRxBuf, &esp_task_context, "ESP32");
}

/**
 * static void router_from_wifi(void *_param)
{
  xEventGroupSetBits(startUpEventGroup, START_UP_WIFI_ROUTER_RUNNING);
  route(wifi_transport_receive, &wifiRxBuf, &wifi_task_context, "HOST");
}

 */

static void router_from_wifi_peer(void *_param)
{ // <— NUOVO
  xEventGroupSetBits(startUpEventGroup, START_UP_WIFI_PEER_ROUTER_RUNNING);
  route(wifi_peer_receive_and_fix, &wifiPeerRxBuf, &wifi_peer_task_context, "WIFI_PEER");
}

void router_init()
{
  startUpEventGroup = xEventGroupCreate();
  xEventGroupClearBits(startUpEventGroup,
                       START_UP_GAP8_ROUTER_RUNNING |
                           START_UP_CF_ROUTER_RUNNING |
                           START_UP_ESP_ROUTER_RUNNING |
                           // START_UP_WIFI_ROUTER_RUNNING |
                           START_UP_WIFI_PEER_ROUTER_RUNNING);

  xTaskCreate(router_from_gap8, "Router from GAP8", 5000, NULL, 1, NULL);
  xTaskCreate(router_from_crazyflie, "Router from CF", 5000, NULL, 1, NULL);
  xTaskCreate(router_from_esp32, "Router from ESP32", 5000, NULL, 1, NULL);
  // xTaskCreate(router_from_wifi, "Router from WIFI", 5000, NULL, 1, NULL);
  xTaskCreate(router_from_wifi_peer, "Router from WIFI_PEER", 5000, NULL, 1, NULL); // <— NUOVO

  ESP_LOGI("ROUTER", "Waiting for tasks to start");
  xEventGroupWaitBits(startUpEventGroup,
                      START_UP_GAP8_ROUTER_RUNNING |
                          START_UP_CF_ROUTER_RUNNING |
                          START_UP_ESP_ROUTER_RUNNING |
                          // START_UP_WIFI_ROUTER_RUNNING |
                          START_UP_WIFI_PEER_ROUTER_RUNNING,
                      pdTRUE, pdTRUE, portMAX_DELAY);

  ESP_LOGI("ROUTER", "Initialized");
}
