#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app.h"

#include "commander.h"

#include "FreeRTOS.h"
#include "task.h"

#include "debug.h"

#include "param.h"
#include "log.h"

// my stuff
#include "estimator_kalman.h"
#include "radiolink.h"
#include "led.h"


//#define DEBUG_MODULE "PUSH"

typedef struct _DronePosition
{
  unsigned char id;
  float x;
  float y;
  float z;
} DronePosition; // size: 4 + 12 bytes


static DronePosition otherPositions[10];
static DronePosition receivedPosition;


void p2pcallbackHandler(P2PPacket *p)
{
  memcpy(&receivedPosition, p->data, sizeof(DronePosition));
  otherPositions[receivedPosition.id] = receivedPosition;
}

void init()
{
  // put DronePosition structs with out of bounds values into the otherPositions array
  int size = sizeof(otherPositions) / sizeof(otherPositions[0]);
  for (int i = 0; i < size; i++)
  {
    DronePosition dummy = {.id = i, .x = 99999, .y = 99999, .z = 99999};
    otherPositions[i] = dummy;
  }
}

static void setHoverSetpoint(setpoint_t *setpoint, float x, float y, float z)
{
  setpoint->mode.x = modeAbs;
  setpoint->mode.y = modeAbs;
  setpoint->mode.z = modeAbs;
  setpoint->mode.yaw = modeAbs;

  setpoint->position.x = x;
  setpoint->position.y = y;
  setpoint->position.z = z;
  setpoint->attitude.yaw = 0;
}

// typedef enum
// {
//   idle,
//   lowUnlock,
//   unlocked,
//   stopping
// } State;

// static State state = idle;

// #define MAX(a, b) ((a > b) ? a : b)
// #define MIN(a, b) ((a < b) ? a : b)

void appMain()
{
  static uint8_t droneID = 0;
  static uint8_t droneAmount = 0;
  PARAM_GROUP_START(drone)
  PARAM_ADD(PARAM_UINT8, id, &droneID)
  PARAM_ADD(PARAM_UINT8, amount, &droneAmount)
  PARAM_GROUP_STOP(drone)

  static int8_t cmd = 0;
  PARAM_GROUP_START(cmd)
  PARAM_ADD(PARAM_INT8, cmd, &cmd)
  PARAM_GROUP_STOP(cmd)

  static int8_t send = 0;
  PARAM_GROUP_START(p2p)
  PARAM_ADD(PARAM_UINT8, send, &send)
  PARAM_GROUP_STOP(p2p)


  static float dbgflt = 0;
  static uint8_t dbgchr = 0;
  PARAM_GROUP_START(dbg)
  PARAM_ADD(PARAM_FLOAT, flt, &dbgflt)
  PARAM_ADD(PARAM_UINT8, chr, &dbgchr)
  PARAM_GROUP_STOP(dbg)
  LOG_GROUP_START(dbg)
  LOG_ADD(LOG_FLOAT, flt, &dbgflt)
  PARAM_ADD(LOG_UINT8, chr, &dbgchr)
  LOG_GROUP_STOP(dbg)


  static setpoint_t setpoint;
  static point_t kalmanPosition;

  static DronePosition dronePosition = {9, 43.75, -3.6745, 5.8};

  static P2PPacket pk;
  pk.port = 0;

  // vTaskDelay(M2T(500)); // wait x ms, M2T: ms to os ticks
  // resetEstimator();
  // vTaskDelay(M2T(500));


  DEBUG_PRINT("Waiting for activation ...\n");

  init();
  p2pRegisterCB(p2pcallbackHandler);
  
  while (1)
  {
    vTaskDelay(M2T(10));

    // get current position
    estimatorKalmanGetEstimatedPos(&kalmanPosition);

    switch (cmd)
    {
      case 1:
        dbgchr = droneID;
        break;
      case 2:
        dbgflt = receivedPosition.x;
        break;
      case 3:
        dbgflt = receivedPosition.y;
        break;
      case 4:
        dbgflt = receivedPosition.z;
        break;
      case 101:
        pk.size = sizeof(DronePosition);
        memcpy(pk.data, &dronePosition, sizeof(DronePosition));
        radiolinkSendP2PPacketBroadcast(&pk);
        break;
      default:
        break;
    }
    cmd = 0;

    if (1)
    {
      setHoverSetpoint(&setpoint, 0, 0, .3);
    }
  }
}

void waitForPositionEstimator()
{
}

// ledClearAll();
// ledSetAll();
// ledSet(LED_BLUE_L, true);
