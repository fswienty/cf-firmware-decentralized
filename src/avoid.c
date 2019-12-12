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

#define OTHERDRONESAMOUNT 10  // size of the array containing the positions of other drones. must be at least as high as the highest id among all drones.
#define DUMMYPOSITION 99999  // the xyz coordinates of non-connected drones will be set to this value

//#define DEBUG_MODULE "PUSH

typedef enum
{
  uninitialized,
  enginesOff,
  starting,
  landing,
  flying,
  debug1,
  debug2,
} State;

typedef struct _DroneData
{
  uint8_t id;
  Vector3 pos;
} DroneData;  // size: 4 + 12 bytes

static DroneData droneData;  // contains this drones's id and position
static Vector3 targetPosition;  // this drones's target position
static uint8_t receivedDroneId;  // id of the last received droneData
static uint8_t triggerDroneId;  // receiving droneData with this id triggers this drone to transmit its own droneData. set during initialization, don't change at runtime.
static DroneData othersData[OTHERDRONESAMOUNT];  // array of the droneData of the other drones
static P2PPacket pk;  // the radio packet that is send to other drones

void p2pCallbackHandler(P2PPacket *p)
{
  DroneData receivedDroneData;
  memcpy(&receivedDroneData, p->data, sizeof(DroneData));
  receivedDroneId = receivedDroneData.id;
  othersData[receivedDroneData.id] = receivedDroneData;
}

static void communicate()
{
  if (receivedDroneId == triggerDroneId)
  {
    memcpy(pk.data, &droneData, sizeof(DroneData));
    radiolinkSendP2PPacketBroadcast(&pk);
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

static bool checkDistances()
{
  Vector3 dronePosition = droneData.pos;
  bool otherIsClose = false;
  for (int i = 0; i < OTHERDRONESAMOUNT; i++)
  {
    Vector3 otherPosition = othersData[i].pos;
    Vector3 droneToOther = sub(dronePosition, otherPosition);
    float distance = magnitude(droneToOther);
    if (distance < 0.3f)
    {
      otherIsClose = true;
    }
  }
  return otherIsClose;
}

static void approachTarget(setpoint_t *sp)
{
  Vector3 dronePosition = droneData.pos;
  Vector3 droneToTarget = sub(dronePosition, targetPosition);
  float distance = magnitude(droneToTarget);
  distance = MIN(distance, 0.2f);
  droneToTarget = norm(droneToTarget);
  Vector3 waypoint = add(dronePosition, mul(droneToTarget, distance));
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

// ENTRY POINT
void appMain()
{
  static point_t kalmanPosition;
  static setpoint_t setpoint;
  static State state = uninitialized;

  static uint8_t droneAmount = 0;
  // drone.cmd value meanings:
  // 100: used to trigger initialization
  // 1:  start
  // 2:  land
  // 3:  debug1
  // 4:  debug2
  // be careful not to use these values for something else 
  static int8_t droneCmd = 0;
  PARAM_GROUP_START(drone)
  PARAM_ADD(PARAM_UINT8, amount, &droneAmount)
  PARAM_ADD(PARAM_UINT8, id, &droneData.id)
  PARAM_ADD(PARAM_UINT8, triggerId, &triggerDroneId)
  PARAM_ADD(PARAM_INT8, cmd, &droneCmd)
  PARAM_ADD(PARAM_FLOAT, targetX, &targetPosition.x)
  PARAM_ADD(PARAM_FLOAT, targetY, &targetPosition.y)
  PARAM_ADD(PARAM_FLOAT, targetZ, &targetPosition.z)
  PARAM_GROUP_STOP(drone)

  // debug variables which can be written and read from the pc and the drone
  static float dbgflt = 0;
  static uint8_t dbgchr = 0;
  static int dbgint = 0;
  PARAM_GROUP_START(dbg)
  PARAM_ADD(PARAM_FLOAT, flt, &dbgflt)
  PARAM_ADD(PARAM_UINT8, chr, &dbgchr)
  PARAM_ADD(PARAM_INT32, int, &dbgint)
  PARAM_GROUP_STOP(dbg)
  LOG_GROUP_START(dbg)
  LOG_ADD(LOG_FLOAT, flt, &dbgflt)
  PARAM_ADD(LOG_UINT8, chr, &dbgchr)
  PARAM_ADD(LOG_INT32, int, &dbgint)
  LOG_GROUP_STOP(dbg)

  p2pRegisterCB(p2pCallbackHandler);

  while (1)
  {
    vTaskDelay(M2T(10));

    // don't execute the entire while loop before initialization happend
    if (state == uninitialized)
    {
      if (droneCmd == 100)
      {
        // INIT
        receivedDroneId = 255;
        pk.port = 0;

        // put DroneData structs with out of bounds values into the othersData array
        for (int i = 0; i < OTHERDRONESAMOUNT; i++)
        {
          DroneData dummy;
          dummy.id = i;
          dummy.pos.x = DUMMYPOSITION;
          dummy.pos.y = DUMMYPOSITION;
          dummy.pos.z = DUMMYPOSITION;
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
      case 3:
        state = debug1;
        break;
      case 4:
        state = debug2;
        break;
      case 50:
        memcpy(pk.data, &droneData, sizeof(DroneData));
        radiolinkSendP2PPacketBroadcast(&pk);
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
        communicate();
        approachTarget(&setpoint);
        break;
      case debug1:
        communicate();
        if (checkDistances())
        {
          ledSetAll();
        }
        else
        {
          ledClearAll();
        }   
        break;
      case debug2:
        break;
    }
    commanderSetSetpoint(&setpoint, 3);
  }
}


// SOME OLD STUFF

// ledClearAll();
// ledSetAll();
// ledSet(LED_BLUE_L, true);

//int size = sizeof(array) / sizeof(array[0]);
