/*
 * (tue licenze/origine)
 */

#include "stdio.h"
/* PMSIS includes */
#include "pmsis.h"
#include "bsp/buffer.h"

/* PMSIS BSP includes */
#include "bsp/bsp.h"
#include "bsp/buffer.h"
#include "bsp/ai_deck.h"
#include "bsp/camera/himax.h"
#include "FreeRTOS.h"

/* Gaplib includes */
// #include "gaplib/ImgIO.h"

#include "cpx.h"
#include "wifi.h"

// All includes for facedetector application
#include "faceDet.h"
#include "FaceDetKernels.h"
#include "ImageDraw.h"
#include "setup.h"
#include <stdint.h>
#include <stdbool.h>

#define CAM_WIDTH 324
#define CAM_HEIGHT 244
#define IMG_ORIENTATION 0x0101

#define IMAGE_OUT_WIDTH 64
#define IMAGE_OUT_HEIGHT 48

static EventGroupHandle_t evGroup;
#define CAPTURE_DONE_BIT (1 << 0)

#define MY_DRONE_ID 1

// --- NEW --- peer (drone-2) IP/porta e opcodes controllo
#define PEER_IP_BE ((uint32_t)0x0A000066) // 10.0.0.102 in big-endian
#define PEER_PORT_BE ((uint16_t)0x1092)   // htons(4242)

// --- NEW --- finestra minima tra due alert (ms) e timeout salita link
#define ALERT_COOLDOWN_MS 10000u
#define PEER_UP_TIMEOUT_MS 2000u

// Performance menasuring variables
static uint32_t start = 0;
static uint32_t captureTime = 0;
static uint32_t transferTime = 0;
static uint32_t encodingTime = 0;

static int wifiConnected = 0;
static int wifiClientConnected = 0;
static int wifiPeerConnected = 0;
static int peerAckSeen = 0;

static pi_task_t task1;

static CPXPacket_t txp;
static CPXPacket_t rxp;
static CPXPacket_t rxp2;

// --- NEW --- util tempo (ms)
static inline uint32_t now_ms(void)
{
  return pi_time_get_us() / 1000u;
}

// --- Message types standardizzati ---
typedef enum
{
  MSG_TYPE_ALERT = 0,
  MSG_TYPE_ACK,
  MSG_TYPE_CMD_RESPONSE
} MsgType_t;

// Mappa enum -> stringa
static const char *msgTypeToStr(MsgType_t type)
{
  switch (type)
  {
  case MSG_TYPE_ALERT:
    return "alert";
  case MSG_TYPE_ACK:
    return "ack";
  case MSG_TYPE_CMD_RESPONSE:
    return "cmd_response";
  default:
    return "unknown";
  }
}

// Helper JSON unica
static void sendJsonMsg(CPXPacket_t *packet, MsgType_t type, const char *message, int count)
{
  char msg[128];
  int len;

  if (count >= 0)
  {
    len = snprintf(msg, sizeof(msg),
                   "{\"drone_id\":\"drone-%d\",\"type\":\"%s\",\"message\":\"%s\",\"count\":%d}\n",
                   MY_DRONE_ID, msgTypeToStr(type), message, count);
  }
  else
  {
    len = snprintf(msg, sizeof(msg),
                   "{\"drone_id\":\"drone-%d\",\"type\":\"%s\",\"message\":\"%s\"}\n",
                   MY_DRONE_ID, msgTypeToStr(type), message);
  }

  if (len < 0)
  {
    cpxPrintToConsole(LOG_TO_CRTP, "JSON format error\n");
    return;
  }
  if (len > (int)sizeof(packet->data))
  {
    cpxPrintToConsole(LOG_TO_CRTP, "JSON truncated: %d > %u\n", len, (unsigned)sizeof(packet->data));
    len = sizeof(packet->data);
  }

  memcpy(packet->data, msg, len);
  packet->dataLength = (uint16_t)len;

  cpxSendPacketBlocking(packet);
  cpxPrintToConsole(LOG_TO_CRTP, "[GAP8->PEER] sent: %s", msg);
}

void rx_task(void *parameters)
{
  while (1)
  {

    cpxReceivePacketBlocking(CPX_F_WIFI_CTRL, &rxp); // Blocca fino a ricezione
    WiFiCTRLPacket_t *wifiCtrl = (WiFiCTRLPacket_t *)rxp.data;

    switch (wifiCtrl->cmd)
    {
    case WIFI_CTRL_STATUS_WIFI_CONNECTED:
      cpxPrintToConsole(LOG_TO_CRTP, "WiFi connected (%u.%u.%u.%u)\n", wifiCtrl->data[0], wifiCtrl->data[1], wifiCtrl->data[2], wifiCtrl->data[3]);
      wifiConnected = 1;
      break;
    case WIFI_CTRL_STATUS_CLIENT_CONNECTED:
      cpxPrintToConsole(LOG_TO_CRTP, "Wifi client connection status: %u\n", wifiCtrl->data[0]);
      wifiClientConnected = wifiCtrl->data[0];
      break;
    case WIFI_CTRL_PEER_STATUS:
    { // drone-to-drone connection status
      uint8_t st = wifiCtrl->data[0];
      wifiPeerConnected = (st == 1);
      if (st == 2)
      {
        cpxPrintToConsole(LOG_TO_CRTP, "Wifi peer status: %u (busy)\n", st);
      }
      else
      {
        cpxPrintToConsole(LOG_TO_CRTP, "Wifi peer status: %u\n", st);
      }
      break;
    }
    default:
      cpxPrintToConsole(LOG_TO_CRTP, "Unknown wifi command: %u\n", wifiCtrl->data[0]);
      break;
    }
  }
}

static void capture_done_cb(void *arg)
{
  xEventGroupSetBits(evGroup, CAPTURE_DONE_BIT);
}

typedef struct
{
  uint8_t magic;
  uint16_t width;
  uint16_t height;
  uint8_t depth;
  uint8_t type;
  uint32_t size;
} __attribute__((packed)) img_header_t;

typedef enum
{
  RAW_ENCODING = 0,
  JPEG_ENCODING = 1
} __attribute__((packed)) StreamerMode_t;

pi_buffer_t header;
uint32_t headerSize;
pi_buffer_t footer;
uint32_t footerSize;
pi_buffer_t jpeg_data;
uint32_t jpegSize;

static StreamerMode_t streamerMode = RAW_ENCODING;

void createImageHeaderPacket(CPXPacket_t *packet, uint32_t imgSize, StreamerMode_t imgType)
{
  img_header_t *imgHeader = (img_header_t *)packet->data;
  imgHeader->magic = 0xBC;
  imgHeader->width = CAM_WIDTH;
  imgHeader->height = CAM_HEIGHT;
  imgHeader->depth = 1;
  imgHeader->type = imgType;
  imgHeader->size = imgSize;
  packet->dataLength = sizeof(img_header_t);
}

void sendBufferViaCPX(CPXPacket_t *packet, uint8_t *buffer, uint32_t bufferSize)
{
  uint32_t offset = 0;
  uint32_t size = 0;
  do
  {
    size = sizeof(packet->data);
    if (offset + size > bufferSize)
    {
      size = bufferSize - offset;
    }
    memcpy(packet->data, &buffer[offset], sizeof(packet->data));
    packet->dataLength = size;
    cpxSendPacketBlocking(packet);
    offset += size;
  } while (size == sizeof(packet->data));
}

// Intializing buffers for camera images
static unsigned char *imgBuff0;
static struct pi_device ili;
static pi_buffer_t buffer;
static struct pi_device cam;

// Initialize buffers for images handled in the cluster
static pi_buffer_t buffer_out;
L2_MEM unsigned char *ImageOut;
L2_MEM unsigned int *ImageIntegral;
L2_MEM unsigned int *SquaredImageIntegral;
L2_MEM char str_to_lcd[100];

// Intialize structures for clusters
struct pi_device cluster_dev;
struct pi_cluster_task *task;
struct pi_cluster_conf conf;
ArgCluster_T ClusterCall;

// Open himax camera funciton
static int open_camera_himax(struct pi_device *device)
{
  struct pi_himax_conf cam_conf;

  pi_himax_conf_init(&cam_conf);

  cam_conf.format = PI_CAMERA_QVGA;

  pi_open_from_conf(device, &cam_conf);
  if (pi_camera_open(device))
    return -1;

  // rotate image
  pi_camera_control(device, PI_CAMERA_CMD_START, 0);
  uint8_t set_value = 3;
  uint8_t reg_value;
  pi_camera_reg_set(device, IMG_ORIENTATION, &set_value);
  pi_time_wait_us(1000000);
  pi_camera_reg_get(device, IMG_ORIENTATION, &reg_value);
  if (set_value != reg_value)
  {
    cpxPrintToConsole(LOG_TO_CRTP, "Failed to rotate camera image\n");
    return -1;
  }
  pi_camera_control(device, PI_CAMERA_CMD_STOP, 0);
  pi_camera_control(device, PI_CAMERA_CMD_AEG_INIT, 0);

  return 0;
}

static int open_camera(struct pi_device *device)
{
  return open_camera_himax(device);
}

// UART init param
L2_MEM struct pi_uart_conf uart_conf;
L2_MEM struct pi_device uart;
L2_MEM uint8_t rec_digit = -1;

void facedetection_task(void *parameters)
{
  vTaskDelay(2000);
  cpxInitRoute(CPX_T_GAP8, CPX_T_WIFI_HOST, CPX_F_APP, &txp.route);
  cpxPrintToConsole(LOG_TO_CRTP, "Starting face detection task...\n");

  unsigned int W = CAM_WIDTH, H = CAM_HEIGHT;
  unsigned int Wout = 64, Hout = 48;
  unsigned int ImgSize = W * H;

  imgBuff0 = (unsigned char *)pmsis_l2_malloc((CAM_WIDTH * CAM_HEIGHT) * sizeof(unsigned char));
  if (imgBuff0 == NULL)
  {
    cpxPrintToConsole(LOG_TO_CRTP, "Failed to allocate Memory for Image \n");
    pmsis_exit(-1);
  }

  // Malloc up image buffers to be used in the cluster
  ImageOut = (unsigned char *)pmsis_l2_malloc((Wout * Hout) * sizeof(unsigned char));
  ImageIntegral = (unsigned int *)pmsis_l2_malloc((Wout * Hout) * sizeof(unsigned int));
  SquaredImageIntegral = (unsigned int *)pmsis_l2_malloc((Wout * Hout) * sizeof(unsigned int));
  if (ImageOut == 0)
  {
    cpxPrintToConsole(LOG_TO_CRTP, "Failed to allocate Memory for Image (%d bytes)\n", ImgSize * sizeof(unsigned char));
    pmsis_exit(-2);
  }
  if ((ImageIntegral == 0) || (SquaredImageIntegral == 0))
  {
    cpxPrintToConsole(LOG_TO_CRTP, "Failed to allocate Memory for one or both Integral Images (%d bytes)\n", ImgSize * sizeof(unsigned int));
    pmsis_exit(-3);
  }

  if (open_camera(&cam))
  {
    cpxPrintToConsole(LOG_TO_CRTP, "Failed to open camera\n");
    pmsis_exit(-5);
  }

  //  UART init with Crazyflie and configure
  pi_uart_conf_init(&uart_conf);
  uart_conf.enable_tx = 1;
  uart_conf.enable_rx = 0;
  pi_open_from_conf(&uart, &uart_conf);
  if (pi_uart_open(&uart))
  {
    cpxPrintToConsole(LOG_TO_CRTP, "[UART] open failed !\n");
    pmsis_exit(-1);
  }
  cpxPrintToConsole(LOG_TO_CRTP, "[UART] Open\n");

  // Setup buffer for images
  buffer.data = imgBuff0 + CAM_WIDTH * 2 + 2;
  buffer.stride = 4;

  // WIth Himax, propertly configure the buffer to skip boarder pixels
  pi_buffer_init(&buffer, PI_BUFFER_TYPE_L2, imgBuff0 + CAM_WIDTH * 2 + 2);
  pi_buffer_set_stride(&buffer, 4);
  pi_buffer_set_format(&buffer, CAM_WIDTH, CAM_HEIGHT, 1, PI_BUFFER_FORMAT_GRAY);

  buffer_out.data = ImageOut;
  buffer_out.stride = 0;
  pi_buffer_init(&buffer_out, PI_BUFFER_TYPE_L2, ImageOut);
  pi_buffer_set_format(&buffer_out, IMAGE_OUT_WIDTH, IMAGE_OUT_HEIGHT, 1, PI_BUFFER_FORMAT_GRAY);
  pi_buffer_set_stride(&buffer_out, 0);

  // Assign pointers for cluster structure
  ClusterCall.ImageIn = imgBuff0;
  ClusterCall.Win = W;
  ClusterCall.Hin = H;
  ClusterCall.Wout = Wout;
  ClusterCall.Hout = Hout;
  ClusterCall.ImageOut = ImageOut;
  ClusterCall.ImageIntegral = ImageIntegral;
  ClusterCall.SquaredImageIntegral = SquaredImageIntegral;

  pi_cluster_conf_init(&conf);
  pi_open_from_conf(&cluster_dev, (void *)&conf);
  pi_cluster_open(&cluster_dev);

  pi_freq_set(PI_FREQ_DOMAIN_CL, 75000000);

  // Send intializer function to cluster
  task = (struct pi_cluster_task *)pmsis_l2_malloc(sizeof(struct pi_cluster_task));
  memset(task, 0, sizeof(struct pi_cluster_task));
  task->entry = (void *)faceDet_cluster_init;
  task->arg = &ClusterCall;
  pi_cluster_send_task_to_cl(&cluster_dev, task);

  // Assign function for main cluster loop
  task->entry = (void *)faceDet_cluster_main;
  task->arg = &ClusterCall;

  int nb_frames = 0;
  EventBits_t evBits;
  pi_camera_control(&cam, PI_CAMERA_CMD_STOP, 0);

  // --- NEW --- throttling alert
  uint32_t lastAlertMs = 0;

  while (1 && (NB_FRAMES == -1 || nb_frames < NB_FRAMES))
  {
    // Capture image
    pi_camera_capture_async(&cam, imgBuff0, CAM_WIDTH * CAM_HEIGHT, pi_task_callback(&task1, capture_done_cb, NULL));
    pi_camera_control(&cam, PI_CAMERA_CMD_START, 0);
    evBits = xEventGroupWaitBits(evGroup, CAPTURE_DONE_BIT, pdTRUE, pdFALSE, (TickType_t)(500 / portTICK_PERIOD_MS));
    pi_camera_control(&cam, PI_CAMERA_CMD_STOP, 0);
    while ((evBits & CAPTURE_DONE_BIT) != CAPTURE_DONE_BIT)
    {
      cpxPrintToConsole(LOG_TO_CRTP, "Failed camera acquisition\n");
      pi_camera_control(&cam, PI_CAMERA_CMD_START, 0);
      evBits = xEventGroupWaitBits(evGroup, CAPTURE_DONE_BIT, pdTRUE, pdFALSE, (TickType_t)(500 / portTICK_PERIOD_MS));
      pi_camera_control(&cam, PI_CAMERA_CMD_STOP, 0);
    }

    // Run detection
    pi_cluster_send_task_to_cl(&cluster_dev, task);

    /**
     *     // --- MOD --- invio: oltre all'HOST, aggiungiamo il canale DRONE (con cooldown e connect/close)
        if (ClusterCall.num_reponse > 0)
        {
          // (a) invio verso PC host come prima, se collegato
          if (wifiClientConnected == 1)
          {
            sendJsonMsg(&txp, MSG_TYPE_ALERT, "face detected", ClusterCall.num_reponse);
          }

          // (b) invio verso DRONE-2 con rate-limit 10s
          uint32_t now = now_ms();
          if (lastAlertMs == 0 || (now - lastAlertMs) >= ALERT_COOLDOWN_MS)
          {
            // i) chiedi apertura connessione peer
            CPXPacket_t txCtrl = (CPXPacket_t){0}; // controllo
            cpxInitRoute(CPX_T_GAP8, CPX_T_ESP32, CPX_F_WIFI_CTRL, &txCtrl.route);
            txCtrl.data[0] = WIFI_CTRL_PEER_CONNECT;
            uint32_t ip_be = PEER_IP_BE;
            uint16_t port_be = PEER_PORT_BE;
            memcpy(&txCtrl.data[1], &ip_be, 4);
            memcpy(&txCtrl.data[5], &port_be, 2);
            txCtrl.dataLength = 1 + 4 + 2;
            cpxSendPacketBlocking(&txCtrl);

            // ii) attendi link UP (fino a 2s)
            uint32_t deadline = now_ms() + PEER_UP_TIMEOUT_MS;
            while (!wifiPeerConnected && now_ms() < deadline)
            {
              vTaskDelay(1);
            }

            if (wifiPeerConnected)
            {
              // iii) prepara pacchetto dati verso PEER
              CPXPacket_t txPeer = (CPXPacket_t){0};
    #ifndef CPX_T_WIFI_PEER
    #define CPX_T_WIFI_PEER ((CPXTarget_t)5) // A remote computer connected via Wifi peer-to-peer
    #endif

              cpxInitRoute(CPX_T_GAP8, CPX_T_WIFI_PEER, CPX_F_APP, &txPeer.route);
              sendJsonMsg(&txPeer, MSG_TYPE_ALERT, "face detected", ClusterCall.num_reponse);

              peerAckSeen = 0;
              uint32_t ackDeadline = now_ms() + 500;
              while (!peerAckSeen && now_ms() < ackDeadline)
              {
                vTaskDelay(1);
              }
              if (!peerAckSeen)
              {
                cpxPrintToConsole(LOG_TO_CRTP, "Warning: no ACK received from peer! (timeout)\n");
              }

              // iv) chiudi la sessione per liberare il peer
              CPXPacket_t txClose = (CPXPacket_t){0};
              cpxInitRoute(CPX_T_GAP8, CPX_T_ESP32, CPX_F_WIFI_CTRL, &txClose.route);
              txClose.data[0] = WIFI_CTRL_PEER_CLOSE;
              txClose.dataLength = 1;
              cpxSendPacketBlocking(&txClose);

              lastAlertMs = now_ms();
            }
            else
            {
              cpxPrintToConsole(LOG_TO_CRTP, "Peer not UP: skipped peer alert\n");
            }
          }
        }

     */
    // Manda il numero di facce via UART (come prima)
    pi_uart_write(&uart, &ClusterCall.num_reponse, 1);

    nb_frames++;
  }
  cpxPrintToConsole(LOG_TO_CRTP, "Test face detection done.\n");
  pmsis_exit(0);
}

static int parseType(const char *json, const char *typeToCheck)
{
  const char *p = json;

  while (*p)
  {
    // cerca la sottostringa "type"
    if (p[0] == '"' && p[1] == 't' && p[2] == 'y' && p[3] == 'p' && p[4] == 'e' && p[5] == '"')
    {
      // cerca i due punti
      const char *colon = strchr(p, ':');
      if (colon)
      {
        // cerca la prima doppia virgoletta dopo il :
        const char *start = strchr(colon, '"');
        if (start)
        {
          start++; // subito dopo il "
          const char *end = strchr(start, '"');
          if (end)
          {
            int len = end - start;
            if (strncmp(start, typeToCheck, len) == 0 && typeToCheck[len] == '\0')
            {
              return 1; // trovato type uguale
            }
          }
        }
      }
    }
    p++;
  }
  return 0; // non trovato
}

void rx_client_msg(void *parameters)
{
  cpxEnableFunction(CPX_F_APP);
  cpxPrintToConsole(LOG_TO_CRTP, "Starting client message receiver\n");

  static CPXPacket_t rx;
  static CPXPacket_t tx; // per la risposta ACK

  // --- MOD --- la route di risposta verrà decisa in base al mittente (HOST o PEER)
  // cpxInitRoute(CPX_T_GAP8, CPX_T_WIFI_HOST, CPX_F_APP, &tx.route);

  while (1)
  {
    // 1) ricezione bloccante
    cpxReceivePacketBlocking(CPX_F_APP, &rx);

    // 2) log del payload ricevuto (safe string)
    char buf[257];
    int copy_len = (rx.dataLength < 256) ? rx.dataLength : 256;
    memcpy(buf, rx.data, copy_len);
    buf[copy_len] = '\0';
    cpxPrintToConsole(LOG_TO_CRTP, "[APP->GAP8] msg: %s\n", buf);

    // se arriva un ACK dal peer, segno la bandierina

    if (rx.route.source != CPX_T_WIFI_HOST)
    {

      if (parseType(buf, "ack"))
      {
        peerAckSeen = 1;
        cpxPrintToConsole(LOG_TO_CRTP, "Ricevuto ACK! (no reply)\n");
        continue; // non rispondo all'ACK del peer
      }
    }

    memset(&tx, 0, sizeof(tx));
    if (rx.route.source == CPX_T_WIFI_HOST)
    {
      cpxInitRoute(CPX_T_GAP8, CPX_T_WIFI_HOST, CPX_F_APP, &tx.route);
      sendJsonMsg(&tx, MSG_TYPE_ACK, "messaggio ricevuto (host)", -1);
    }
    else
    {
      cpxInitRoute(CPX_T_GAP8, CPX_T_WIFI_PEER, CPX_F_APP, &tx.route);
      sendJsonMsg(&tx, MSG_TYPE_ACK, "messaggio ricevuto (peer)", -1);
    }
  }
}

void start_example(void)
{
  cpxInit();
  cpxEnableFunction(CPX_F_WIFI_CTRL);
  cpxPrintToConsole(LOG_TO_CRTP, "-- WiFi Face Detection started --\n");

  evGroup = xEventGroupCreate();

  BaseType_t xTask;

  xTask = xTaskCreate(rx_task, "rx_task", configMINIMAL_STACK_SIZE * 2,
                      NULL, tskIDLE_PRIORITY + 1, NULL);
  if (xTask != pdPASS)
  {
    cpxPrintToConsole(LOG_TO_CRTP, "RX task did not start!\n");
    pmsis_exit(-1);
  }

  xTask = xTaskCreate(rx_client_msg, "rx_client_msg", configMINIMAL_STACK_SIZE * 2,
                      NULL, tskIDLE_PRIORITY + 1, NULL);
  if (xTask != pdPASS)
  {
    cpxPrintToConsole(LOG_TO_CRTP, "rx_client_msg task did not start!\n");
    pmsis_exit(-1);
  }
  /**
   *
    xTask = xTaskCreate(facedetection_task, "facedetection_task", configMINIMAL_STACK_SIZE * 4,
                        NULL, tskIDLE_PRIORITY + 1, NULL);

    if (xTask != pdPASS)
    {
      cpxPrintToConsole(LOG_TO_CRTP, "Camera task did not start !\n");
      pmsis_exit(-1);
    }

   */
  while (1)
  {
    pi_yield();
  }
}

int main(void)
{
  pi_bsp_init();

  // Increase the FC freq to 250 MHz
  pi_freq_set(PI_FREQ_DOMAIN_FC, 250000000);
  __pi_pmu_voltage_set(PI_PMU_DOMAIN_FC, 1200);
  return pmsis_kickoff((void *)start_example);
}
