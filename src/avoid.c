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
#include "vector3.h"

#define MAX(a, b) ((a > b) ? a : b)
#define MIN(a, b) ((a < b) ? a : b)

//#define DEBUG_MODULE "PUSH

typedef enum
{
  uninitialized,
  enginesOff,
  starting,
  landing,
  flying,
} State;

typedef struct _DroneData
{
  unsigned char id;
  Vector3 pos;
} DroneData;  // size: 4 + 12 bytes

static DroneData droneData;  // this quadcopter's id and position
static Vector3 targetPosition;  // this quadcopter's target position (also id, but it's not needed here)
static DroneData othersData[10];  // array of the droneData of the other quadcopters

void p2pcallbackHandler(P2PPacket *p)
{
  DroneData receivedPosition;
  memcpy(&receivedPosition, p->data, sizeof(DroneData));
  othersData[receivedPosition.id] = receivedPosition;
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

static void approachTarget(setpoint_t *sp)
{
  Vector3 dronePosition = droneData.pos;
  Vector3 droneToTarget = sub(dronePosition, targetPosition);
  float distance = magnitude(droneToTarget);
  distance = MIN(distance, 0.2f);
  droneToTarget = norm(droneToTarget);
  Vector3 waypoint = add(dronePosition, mul(droneToTarget, distance));
  // Vector3 waypoint = targetPosition;
  setHoverSetpoint(sp, waypoint.x, waypoint.y, waypoint.z);
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



void appMain()
{
  static point_t kalmanPosition;
  static setpoint_t setpoint;

  static State state = uninitialized;
  static P2PPacket pk;

  static uint8_t droneID = 0;
  static uint8_t droneAmount = 0;
  // drone.cmd value meanings:
  // 42: used to trigger initialization
  // 1:  start
  // 2:  land
  // be careful not to use these values for something else 
  static int8_t droneCmd = 0;
  PARAM_GROUP_START(drone)
  PARAM_ADD(PARAM_UINT8, id, &droneID)
  PARAM_ADD(PARAM_UINT8, amount, &droneAmount)
  PARAM_ADD(PARAM_INT8, cmd, &droneCmd)
  PARAM_ADD(PARAM_FLOAT, targetX, &targetPosition.x)
  PARAM_ADD(PARAM_FLOAT, targetY, &targetPosition.y)
  PARAM_ADD(PARAM_FLOAT, targetZ, &targetPosition.z)
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

  p2pRegisterCB(p2pcallbackHandler);

  while (1)
  {
    vTaskDelay(M2T(10));

    // if (state == uninitialized && droneCmd == 42)
    // {
    //   init(droneID);
    //   dbgchr = droneData.id;
    //   state = enginesOff;
    //   droneCmd = 0;
    // }

    // don't execute the entire while loop before initialization happend
    if (state == uninitialized)
    {
      if (droneCmd == 42)
      {
        // INIT
        droneData.id = droneID;
        pk.port = 0;

        // put DroneData structs with out of bounds values into the othersData array
        int size = sizeof(othersData) / sizeof(othersData[0]);
        for (int i = 0; i < size; i++)
        {
          DroneData dummy;
          dummy.id = i;
          dummy.pos.x = 99999;
          dummy.pos.y = 99999;
          dummy.pos.z = 99999;
          othersData[i] = dummy;
        }
        dbgchr = droneData.id;
        state = enginesOff;
        droneCmd = 0;
      }
      continue;
    }

    // put kalman position in this drone's droneData struct
    estimatorKalmanGetEstimatedPos(&kalmanPosition);
    droneData.pos.x = kalmanPosition.x;
    droneData.pos.y = kalmanPosition.y;
    droneData.pos.z = kalmanPosition.z;

    switch (droneCmd)
    {
      case 1:
        state = starting;
        break;
      case 2:
        state = landing;
        break;
      case 101:
        pk.size = sizeof(DroneData);
        memcpy(pk.data, &droneData, sizeof(DroneData));
        radiolinkSendP2PPacketBroadcast(&pk);
        dbgchr = pk.size;
        break;
      default:
        break;
    }
    droneCmd = 0;

    switch (state)
    {
      case uninitialized: // this case should never occur since the drone should have been initialized before the switch statement
      case enginesOff:
      default:
        shutOffEngines(&setpoint);
        break;
      case starting:
        moveVertical(&setpoint, 0.4);
        if (droneData.pos.z > 0.7f)
        {
          targetPosition.x = droneData.pos.x;
          targetPosition.y = droneData.pos.y;
          targetPosition.z = droneData.pos.z;
          state = flying;
        }
        break;
      case landing:
        moveVertical(&setpoint, -0.4);
        if (droneData.pos.z < 0.15f)
        {
          state = enginesOff;
        }
        break;
      case flying:
        approachTarget(&setpoint);
        break;
    }
    commanderSetSetpoint(&setpoint, 3);
  }
}

// ledClearAll();
// ledSetAll();
// ledSet(LED_BLUE_L, true);
