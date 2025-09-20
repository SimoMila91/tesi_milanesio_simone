
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
// #include <string.h>

#define CAM_WIDTH 324
#define CAM_HEIGHT 244
#define IMG_ORIENTATION 0x0101

#define IMAGE_OUT_WIDTH 64
#define IMAGE_OUT_HEIGHT 48

static EventGroupHandle_t evGroup;
#define CAPTURE_DONE_BIT (1 << 0)

#define MY_DRONE_ID 1

// IP/porta del peer (drone 2) in big-endian
#define PEER_IP_BE ((uint32_t)0x0A000066) // 10.0.0.102 in big-endian
#define PEER_PORT_BE ((uint16_t)0x1092)   // htons(4242)
#define NO_FACE_IDLE_MS 2000u

// Performance menasuring variables (non usati nella demo)
static uint32_t start = 0;
static uint32_t captureTime = 0;
static uint32_t transferTime = 0;
static uint32_t encodingTime = 0;

// === flag condivisi tra task ===
static volatile int wifiConnected = 0;
static volatile int wifiClientConnected = 0;
static volatile int wifiPeerConnected = 0;
static volatile int wifiPeerBusy = 0; // stato busy del peer (2)
static volatile int peerAckSeen = 0;
// === fine flag ===

// Stato locale volo (per auto-land)
static volatile int in_air = 0;
static volatile uint32_t airborne_until_ms = 0;

// flag per inviare TAKEOFF una sola volta
static int already_took_off = 0;

static uint32_t lastAlertMs = 0;

static pi_task_t task1;

static CPXPacket_t txp;
static CPXPacket_t rxp;

// --- Message types standardizzati ---
typedef enum
{
  MSG_TYPE_ALERT = 0,
  MSG_TYPE_ACK,
  MSG_TYPE_CMD_RESPONSE,
  MSG_TYPE_CMD
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
  case MSG_TYPE_CMD:
    return "cmd";
  default:
    return "unknown";
  }
}

static inline uint32_t now_ms(void) { return pi_time_get_us() / 1000u; }

// ===== JSON helpers =====

static void sendCmdJson(CPXPacket_t *packet, const char *action, float height_m, int duration_ms)
{
  // NB: passo duration_ms = -1 per non includerlo nel JSON
  char msg[160];
  if (!action)
    action = "unknown";

  if (duration_ms >= 0)
  {
    if (height_m == height_m) // check non-NaN
      snprintf(msg, sizeof(msg),
               "{\"drone_id\":\"drone-%d\",\"type\":\"cmd\",\"action\":\"%s\",\"height_m\":%.3f,\"duration_ms\":%d}\n",
               MY_DRONE_ID, action, height_m, duration_ms);
    else
      snprintf(msg, sizeof(msg),
               "{\"drone_id\":\"drone-%d\",\"type\":\"cmd\",\"action\":\"%s\",\"duration_ms\":%d}\n",
               MY_DRONE_ID, action, duration_ms);
  }
  else
  {
    if (height_m == height_m)
      snprintf(msg, sizeof(msg),
               "{\"drone_id\":\"drone-%d\",\"type\":\"cmd\",\"action\":\"%s\",\"height_m\":%.3f}\n",
               MY_DRONE_ID, action, height_m);
    else
      snprintf(msg, sizeof(msg),
               "{\"drone_id\":\"drone-%d\",\"type\":\"cmd\",\"action\":\"%s\"}\n",
               MY_DRONE_ID, action);
  }

  size_t len = strlen(msg);
  if (len > sizeof(packet->data))
    len = sizeof(packet->data);
  memcpy(packet->data, msg, len);
  packet->dataLength = (uint16_t)len;

  cpxSendPacketBlocking(packet);
  cpxPrintToConsole(LOG_TO_CRTP, "[GAP8->APP] sent command: %s", msg);
}

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
    len = sizeof(packet->data);

  memcpy(packet->data, msg, len);
  packet->dataLength = (uint16_t)len;

  cpxSendPacketBlocking(packet);
  cpxPrintToConsole(LOG_TO_CRTP, "[GAP8->APP] sent: %s", msg);
}

// --- strstr (senza <string.h>) ---
static const char *find_substr(const char *hay, const char *needle)
{
  if (!*needle)
    return hay;
  const char *h = hay;
  while (*h)
  {
    if (*h == *needle)
    {
      const char *p = h;
      const char *q = needle;
      while (*p && *q && *p == *q)
      {
        p++;
        q++;
      }
      if (*q == '\0')
        return h; // trovato
    }
    h++;
  }
  return 0;
}

// --- confronto stringhe ---
static int streq(const char *a, const char *b)
{
  while (*a && *b)
  {
    if (*a != *b)
      return 0;
    a++;
    b++;
  }
  return (*a == '\0' && *b == '\0');
}

static int parseJsonStringField(const char *json, const char *key, char *out, int outSize)
{
  const char *p = json;
  int klen = 0;
  while (key[klen] != '\0')
    klen++; // strlen(key)

  while ((p = find_substr(p, key)) != 0)
  {
    // verifico che sia una chiave JSON "key"
    if (p > json && p[-1] != '"')
    {
      p++;
      continue;
    }

    const char *q = p + klen;
    if (*q == '"')
      q++; // gestisco "key"" :
    while (*q == ' ' || *q == '\t')
      q++;
    if (*q != ':')
    {
      p++;
      continue;
    }
    q++;
    while (*q == ' ' || *q == '\t')
      q++;
    if (*q != '"')
    {
      p++;
      continue;
    } // mi aspetto valore stringa
    q++;

    const char *end = q;
    while (*end && *end != '"')
      end++;
    if (!*end)
      return 0; // stringa non chiusa

    int n = (int)(end - q);
    if (n >= outSize)
      n = outSize - 1;
    for (int i = 0; i < n; i++)
      out[i] = q[i];
    out[n] = '\0';
    return 1;
  }
  return 0;
}

static int parseType(const char *json, const char *typeToCheck)
{
  const char *p = json;
  while (*p)
  {
    if (p[0] == '"' && p[1] == 't' && p[2] == 'y' && p[3] == 'p' && p[4] == 'e' && p[5] == '"')
    {
      const char *colon = strchr(p, ':');
      if (colon)
      {
        const char *start = strchr(colon, '"');
        if (start)
        {
          start++;
          const char *end = strchr(start, '"');
          if (end)
          {
            int len = (int)(end - start);
            if (strncmp(start, typeToCheck, len) == 0 && typeToCheck[len] == '\0')
              return 1;
          }
        }
      }
    }
    p++;
  }
  return 0;
}

// estraggo "drone_id":"drone-X" e ritorno X, oppure -1
static int parseDroneId(const char *json)
{
  char tmp[32] = {0};
  if (!parseJsonStringField(json, "drone_id", tmp, sizeof(tmp)))
    return -1;
  const char *dash = strchr(tmp, '-');
  if (!dash)
    return -1;
  int id = 0;
  const char *q = dash + 1;
  int ok = 0;
  while (*q >= '0' && *q <= '9')
  {
    id = id * 10 + (*q - '0');
    q++;
    ok = 1;
  }
  return ok ? id : -1;
}

// ===== endian helpers =====
static inline void write_be16(uint8_t *dst, uint16_t v)
{
  dst[0] = (uint8_t)(v >> 8);
  dst[1] = (uint8_t)(v);
}

static inline void write_be32(uint8_t *dst, uint32_t v)
{
  dst[0] = (uint8_t)(v >> 24);
  dst[1] = (uint8_t)(v >> 16);
  dst[2] = (uint8_t)(v >> 8);
  dst[3] = (uint8_t)(v);
}

// Ritorna 1 se connesso (già o dopo richiesta), 0 se non connesso (es. busy o timeout)
static int requestPeerConnectBlocking(uint32_t timeout_ms)
{
  uint32_t start_t = now_ms();
  uint32_t lastTry = 0;

  // se già connesso (stessa sessione/peer), torna 1: non c'è bisogno di inviare nulla
  if (wifiPeerConnected)
    return 1;

  while ((now_ms() - start_t) < timeout_ms)
  {
    if (wifiPeerConnected)
      return 1;

    if (wifiPeerBusy)
    {
      // 2 = occupato con altro peer
      cpxPrintToConsole(LOG_TO_CRTP, "[peer] status=BUSY (2). Abort on-demand connect in this cycle.\n");
      return 0;
    }

    // non busy: provo CONNECT ogni ~300ms
    if ((now_ms() - lastTry) >= 300)
    {
      CPXPacket_t txCtrl = (CPXPacket_t){0};
      cpxInitRoute(CPX_T_GAP8, CPX_T_ESP32, CPX_F_WIFI_CTRL, &txCtrl.route);
      txCtrl.data[0] = WIFI_CTRL_PEER_CONNECT;
      write_be32(&txCtrl.data[1], PEER_IP_BE);
      write_be16(&txCtrl.data[5], PEER_PORT_BE);
      txCtrl.dataLength = 1 + 4 + 2;
      cpxSendPacketBlocking(&txCtrl);
      cpxPrintToConsole(LOG_TO_CRTP, "[peer] CONNECT sent\n");
      lastTry = now_ms();
    }

    vTaskDelay(pdMS_TO_TICKS(20)); // ~50Hz poll dello status arrivato in rx_task
  }

  return wifiPeerConnected ? 1 : 0;
}

// ===== WiFi CTRL RX =====
void rx_task(void *parameters)
{
  while (1)
  {
    cpxReceivePacketBlocking(CPX_F_WIFI_CTRL, &rxp); // blocco fino a ricezione
    WiFiCTRLPacket_t *wifiCtrl = (WiFiCTRLPacket_t *)rxp.data;

    switch (wifiCtrl->cmd)
    {
    case WIFI_CTRL_STATUS_WIFI_CONNECTED:
      cpxPrintToConsole(LOG_TO_CRTP, "WiFi connected (%u.%u.%u.%u)\n",
                        wifiCtrl->data[0], wifiCtrl->data[1], wifiCtrl->data[2], wifiCtrl->data[3]);
      wifiConnected = 1;
      break;
    case WIFI_CTRL_STATUS_CLIENT_CONNECTED:
      cpxPrintToConsole(LOG_TO_CRTP, "Wifi client connection status: %u\n", wifiCtrl->data[0]);
      wifiClientConnected = wifiCtrl->data[0];
      break;
    case WIFI_CTRL_PEER_STATUS:
    {
      // 0=down, 1=up, 2=busy
      uint8_t st = wifiCtrl->data[0];
      wifiPeerConnected = (st == 1);
      wifiPeerBusy = (st == 2);
      if (st == 2)
        cpxPrintToConsole(LOG_TO_CRTP, "Wifi peer status: busy\n");
      else
        cpxPrintToConsole(LOG_TO_CRTP, "Wifi peer status: %u\n", st);
      break;
    }
    default:
      cpxPrintToConsole(LOG_TO_CRTP, "Unknown wifi command (cmd=%u)\n", wifiCtrl->cmd);
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
      size = bufferSize - offset;
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

  // With Himax, properly configure the buffer to skip border pixels
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

  // Send initializer function to cluster
  task = (struct pi_cluster_task *)pmsis_l2_malloc(sizeof(struct pi_cluster_task));
  memset(task, 0, sizeof(struct pi_cluster_task));
  task->entry = (void *)faceDet_cluster_init;
  task->arg = &ClusterCall;
  pi_cluster_send_task_to_cl(&cluster_dev, task);

  // Main cluster loop
  task->entry = (void *)faceDet_cluster_main;
  task->arg = &ClusterCall;

  int nb_frames = 0;
  EventBits_t evBits;
  pi_camera_control(&cam, PI_CAMERA_CMD_STOP, 0);

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

    // invio alert se almeno una faccia
    if (ClusterCall.num_reponse > 0)
    {
      uint32_t now = now_ms();
      if ((now - lastAlertMs) >= NO_FACE_IDLE_MS)
      {
        // 0) Provo ad aprire la connessione verso il peer (max 3s)
        if (!wifiPeerConnected)
        {
          cpxPrintToConsole(LOG_TO_CRTP, "[peer] not connected, trying to connect...\n");
          int ok = requestPeerConnectBlocking(3000);
          cpxPrintToConsole(LOG_TO_CRTP, "[peer] connect result: ok=%d, connected=%d, busy=%d\n",
                            ok, wifiPeerConnected, wifiPeerBusy);
          if (!ok)
          {
            if (wifiPeerBusy)
              cpxPrintToConsole(LOG_TO_CRTP, "[peer] busy (2): skip send this cycle\n");
            else
              cpxPrintToConsole(LOG_TO_CRTP, "[peer] connect timeout: skip send this cycle\n");
          }
        }

        // 1) ALERT a host (se collegato)
        if (wifiClientConnected == 1)
        {
          cpxInitRoute(CPX_T_GAP8, CPX_T_WIFI_HOST, CPX_F_APP, &txp.route);
          sendJsonMsg(&txp, MSG_TYPE_ALERT, "face detected", ClusterCall.num_reponse);
        }

        // 2) ALERT verso peer (solo se connesso)
        if (wifiPeerConnected)
        {
          CPXPacket_t txPeer = (CPXPacket_t){0};
          cpxInitRoute(CPX_T_GAP8, CPX_T_WIFI_PEER, CPX_F_APP, &txPeer.route);
          sendJsonMsg(&txPeer, MSG_TYPE_ALERT, "face detected", ClusterCall.num_reponse);
        }

        lastAlertMs = now;

        // 3) TAKEOFF (una sola volta), solo se peer connesso
        if (wifiPeerConnected && !already_took_off)
        {
          CPXPacket_t txCmd = (CPXPacket_t){0};
          cpxInitRoute(CPX_T_GAP8, CPX_T_WIFI_PEER, CPX_F_APP, &txCmd.route);
          // niente duration_ms nel JSON (passo -1)
          sendCmdJson(&txCmd, "takeoff", 0.5f, -1);
          already_took_off = 1;
        }
      }
    }

    pi_uart_write(&uart, &ClusterCall.num_reponse, 1);

    nb_frames++;
  }
  cpxPrintToConsole(LOG_TO_CRTP, "Test face detection done.\n");
  pmsis_exit(0);
}

// ===== Task di ricezione messaggi da APP (host/peer) =====
void rx_client_msg(void *parameters)
{
  cpxEnableFunction(CPX_F_APP);
  cpxPrintToConsole(LOG_TO_CRTP, "Starting client message receiver\n");

  static CPXPacket_t rx;
  static CPXPacket_t tx;   // per la risposta ACK
  static CPXPacket_t txSt; // per inoltro a STM32

  while (1)
  {
    cpxReceivePacketBlocking(CPX_F_APP, &rx);

    char buf[257];
    int copy_len = (rx.dataLength < 256) ? rx.dataLength : 256;
    memcpy(buf, rx.data, copy_len);
    buf[copy_len] = '\0';
    cpxPrintToConsole(LOG_TO_CRTP, "[APP->GAP8] msg: %s\n", buf);

    // 1) ack dal peer: non rispondo
    if (rx.route.source != CPX_T_WIFI_HOST && parseType(buf, "ack"))
    {
      peerAckSeen = 1;
      cpxPrintToConsole(LOG_TO_CRTP, "Ricevuto ACK!\n");
      continue;
    }

    // 2) comandi (cmd) -> inoltro a STM32 ed eseguo logica locale
    if (parseType(buf, "cmd"))
    {
      char action[32] = {0};
      int senderId = parseDroneId(buf); // -1 se assente

      if (parseJsonStringField(buf, "action", action, sizeof(action)))
      {
        cpxPrintToConsole(LOG_TO_CRTP, "Command action: %s (from drone_id=%d)\n", action, senderId);
      }
      else
      {
        cpxPrintToConsole(LOG_TO_CRTP, "Warning: no action field in cmd\n");
      }

      // Se il comando proviene da me stesso (eco) => ignoro
      if (senderId == MY_DRONE_ID)
      {
        cpxPrintToConsole(LOG_TO_CRTP, "Ignoring self-command (drone_id match)\n");
        continue;
      }

      /**
       * // Inoltro payload allo STM32 (trasparente)
      memset(&txSt, 0, sizeof(txSt));
      cpxInitRoute(CPX_T_GAP8, CPX_T_STM32, CPX_F_APP, &txSt.route);
      int len = rx.dataLength;
      if (len > (int)sizeof(txSt.data))
        len = sizeof(txSt.data);
      memcpy(txSt.data, rx.data, len);
      txSt.dataLength = (uint16_t)len;
      cpxSendPacketBlocking(&txSt);
       */

      // Logica locale per takeoff/land (auto-land a 10s)
      if (streq(action, "takeoff") == 1)
      {
        if (!in_air)
        {
          in_air = 1;
          airborne_until_ms = now_ms() + 10000; // 10 s
          cpxPrintToConsole(LOG_TO_CRTP, "[AIR] takeoff received: in_air=1 until %u ms\n", (unsigned)airborne_until_ms);
        }
        else
        {
          cpxPrintToConsole(LOG_TO_CRTP, "[AIR] takeoff received but already in air, ignoring\n");
        }
      }
      else if (streq(action, "land") == 1)
      {
        in_air = 0; // atterro subito se ricevuto comando esplicito
        cpxPrintToConsole(LOG_TO_CRTP, "[AIR] land received: in_air=0\n");
      }

      // no ack
      continue;
    }

    // 3) altri tipi: ack
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

// ===== Task di gestione auto-land (polling su now_ms) =====
static void air_mgr_task(void *parameters)
{
  (void)parameters;
  while (1)
  {
    if (in_air)
    {
      uint32_t now = now_ms();
      if (now >= airborne_until_ms)
      {
        /**
         *  // invio {"type":"cmd","action":"land"} allo STM32
         CPXPacket_t txLand = (CPXPacket_t){0};
         cpxInitRoute(CPX_T_GAP8, CPX_T_STM32, CPX_F_APP, &txLand.route);
         const char *msg = "{\"type\":\"cmd\",\"action\":\"land\"}\n";
         size_t len = strlen(msg);
         if (len > sizeof(txLand.data))
           len = sizeof(txLand.data);
         memcpy(txLand.data, msg, len);
         txLand.dataLength = (uint16_t)len;
         cpxSendPacketBlocking(&txLand);

         */
        in_air = 0; // reset stato
        cpxPrintToConsole(LOG_TO_CRTP, "[AUTO-LAND] sent to STM32, in_air=0\n");
      }
    }
    vTaskDelay(pdMS_TO_TICKS(20)); // ~50 Hz
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

  // Task per auto-land con polling di now_ms()
  xTask = xTaskCreate(air_mgr_task, "air_mgr_task", configMINIMAL_STACK_SIZE * 2,
                      NULL, tskIDLE_PRIORITY + 1, NULL);
  if (xTask != pdPASS)
  {
    cpxPrintToConsole(LOG_TO_CRTP, "air_mgr_task did not start!\n");
    pmsis_exit(-1);
  }

  xTask = xTaskCreate(facedetection_task, "facedetection_task", configMINIMAL_STACK_SIZE * 4,
                      NULL, tskIDLE_PRIORITY + 1, NULL);
  if (xTask != pdPASS)
  {
    cpxPrintToConsole(LOG_TO_CRTP, "Camera task did not start !\n");
    pmsis_exit(-1);
  }

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
