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


//#define DEBUG_MODULE "PUSH

typedef enum
{
  uninitialized,
  deactivated,
  starting,
  landing,
  idling,
  moving,
} State;

typedef struct _DronePosition
{
  unsigned char id;
  float x;
  float y;
  float z;
} DronePosition; // size: 4 + 12 bytes

static State state = uninitialized;
static setpoint_t setpoint;
static point_t kalmanPosition;
static DronePosition dronePosition = {.id = 0, .x = 99999, .y = 99999, .z = 99999};
static P2PPacket pk;
static DronePosition otherPositions[10];
// static DronePosition receivedPosition;

void p2pcallbackHandler(P2PPacket *p)
{
  DronePosition receivedPosition;
  memcpy(&receivedPosition, p->data, sizeof(DronePosition));
  otherPositions[receivedPosition.id] = receivedPosition;
}

void init(uint8_t droneID)
{
  dronePosition.id = droneID;
  pk.port = 0;

  // put DronePosition structs with out of bounds values into the otherPositions array
  int size = sizeof(otherPositions) / sizeof(otherPositions[0]);
  for (int i = 0; i < size; i++)
  {
    DronePosition dummy = {.id = i, .x = 99999, .y = 99999, .z = 99999};
    otherPositions[i] = dummy;
  }
}

static void setHoverSetpoint(setpoint_t *sp, float x, float y, float z)
{
  sp->mode.x = modeAbs;
  sp->mode.y = modeAbs;
  sp->mode.z = modeAbs;
  sp->mode.yaw = modeAbs;

  sp->position.x = x;
  sp->position.y = y;
  sp->position.z = z;
  sp->attitude.yaw = 0;
}

static void moveVertical(setpoint_t *sp, float zVelocity)
{
  sp->mode.x = modeVelocity;
  sp->mode.y = modeVelocity;
  sp->mode.z = modeVelocity;
  sp->mode.yaw = modeAbs;

  sp->velocity.x = 0.0;
  sp->velocity.y = 0.0;
  sp->velocity.z = zVelocity;
  sp->attitudeRate.yaw = 0.0;
}

static void shutOffEngines(setpoint_t *sp)
{
  sp->mode.x = modeDisable;
  sp->mode.y = modeDisable;
  sp->mode.z = modeDisable;
  sp->mode.yaw = modeDisable;
}

// #define MAX(a, b) ((a > b) ? a : b)
// #define MIN(a, b) ((a < b) ? a : b)

void appMain()
{
  static uint8_t droneInit = 0;
  static uint8_t droneID = 0;
  static uint8_t droneAmount = 0;
  static int8_t droneCmd = 0;
  static int8_t droneFly = 0;
  PARAM_GROUP_START(drone)
  PARAM_ADD(PARAM_UINT8, init, &droneInit)
  PARAM_ADD(PARAM_UINT8, id, &droneID)
  PARAM_ADD(PARAM_UINT8, amount, &droneAmount)
  PARAM_ADD(PARAM_INT8, cmd, &droneCmd)
  PARAM_ADD(PARAM_INT8, fly, &droneFly)
  PARAM_GROUP_STOP(drone)

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

  // vTaskDelay(M2T(500)); // wait x ms, M2T: ms to os ticks
  // resetEstimator();
  // vTaskDelay(M2T(500));

  p2pRegisterCB(p2pcallbackHandler);

  while (1)
  {
    vTaskDelay(M2T(10));

    if (droneInit)
    {
      init(droneID);
      dbgchr = dronePosition.id;
      state = deactivated;
      droneInit = 0;
    }

    if (state == uninitialized)
    {
      continue;
    }


    // put kalman position in this drone's dronePosition struct
    estimatorKalmanGetEstimatedPos(&kalmanPosition);
    dronePosition.x = kalmanPosition.x;
    dronePosition.y = kalmanPosition.y;
    dronePosition.z = kalmanPosition.z;

    switch (droneCmd)
    {
      case 1:
        state = starting;
        break;
      case 2:
        state = landing;
        break;
      case 101:
        pk.size = sizeof(DronePosition);
        memcpy(pk.data, &dronePosition, sizeof(DronePosition));
        radiolinkSendP2PPacketBroadcast(&pk);
        break;
      default:
        break;
    }
    droneCmd = 0;

    DronePosition idlePosition;
    switch (state)
    {
      case uninitialized: // this case should never occur since the drone should have been initialized before this switch
        shutOffEngines(&setpoint);
        break;
      case deactivated:
        shutOffEngines(&setpoint);
        break;
      case starting:
        moveVertical(&setpoint, 0.4);
        if (dronePosition.z > 0.7f)
        {
          idlePosition.id = dronePosition.id;
          idlePosition.x = dronePosition.x;
          idlePosition.y = dronePosition.y;
          idlePosition.z = dronePosition.z;
          state = idling;
        }
        break;
      case landing:
        moveVertical(&setpoint, -0.4);
        if (dronePosition.z < 0.15f)
        {
          state = deactivated;
        }
        break;
      case idling:
        setHoverSetpoint(&setpoint, idlePosition.x, idlePosition.y, idlePosition.z);
        break;
      case moving:
        setHoverSetpoint(&setpoint, 0, 0, 0.1);
        break;
      default:
        shutOffEngines(&setpoint);
        break;
    }
    commanderSetSetpoint(&setpoint, 3);

    // switch (droneFly)
    // {
    //   case 0:
    //     shutOffEngines(&setpoint);
    //     commanderSetSetpoint(&setpoint, 3);
    //     state = deactivated;
    //     break;
    //   case 1:
    //     setHoverSetpoint(&setpoint, 0, 0, .1);
    //     commanderSetSetpoint(&setpoint, 3);
    //     state = flying;
    //     break;
    //   case 2:
    //     setHoverSetpoint(&setpoint, 0, 0, .5);
    //     commanderSetSetpoint(&setpoint, 3);
    //     state = flying;
    //     break;
    // }
  }
}

void waitForPositionEstimator()
{
}

// ledClearAll();
// ledSetAll();
// ledSet(LED_BLUE_L, true);
