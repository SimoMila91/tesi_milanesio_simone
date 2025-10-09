#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app.h"

#include "cpx.h"
#include "cpx_internal_router.h"
#include "ledseq.h"
#include <led.h>

// #include "crtp_commander_high_level.h"
// #include "param.h"
#include "FreeRTOS.h"
#include "task.h"

#define DEBUG_MODULE "APP"
#include "debug.h"

// takeoff su m3
static ledseqStep_t seq_takeoff_def[] = {
  { true,  LEDSEQ_WAITMS(100) },
  { false, LEDSEQ_WAITMS(100) },
  { true,  LEDSEQ_WAITMS(100) },
  { false, LEDSEQ_STOP },
};

static ledseqContext_t seq_takeoff = {
  .sequence = seq_takeoff_def,
  .led = LED_BLUE_L,         
};

// landing su m2
static ledseqStep_t seq_landing_def[] = {
  { true,  LEDSEQ_WAITMS(400) },
  { false, LEDSEQ_STOP },
};

static ledseqContext_t seq_landing = {
  .sequence = seq_landing_def,
  .led = LED_BLUE_NRF,          
};

// callback cpx
static void cpxPacketCallback(const CPXPacket_t* cpxRx);


void appMain(void)
{
  DEBUG_PRINT("App command test started!\n");

  ledseqEnable(true);

  ledseqRegisterSequence(&seq_takeoff);
  ledseqRegisterSequence(&seq_landing); 

  cpxRegisterAppMessageHandler(cpxPacketCallback);
  
}

static void cpxPacketCallback(const CPXPacket_t *cpxRx)
{
    if (cpxRx->data[0] == 1) {
      DEBUG_PRINT("Command received from GAP8: Takeoff\n");
      ledseqRun(&seq_takeoff);
    } else if (cpxRx->data[0] == 2) {
      DEBUG_PRINT("Command received from GAP8: Landing\n"); 
      ledseqRun(&seq_landing);
    } else {
      DEBUG_PRINT("Unknown command\n"); 
    }
}
