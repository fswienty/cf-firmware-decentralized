#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app.h"

#include "commander.h"

#include "FreeRTOS.h"
#include "task.h"

//#include "debug.h"

#include "param.h"
#include "log.h"

// my stuff
#include "estimator_kalman.h"
#include "radiolink.h"
#include "led.h"
#include "vector3.h"
#include "console.h"

#define MAX(a, b) ((a > b) ? a : b)
#define MIN(a, b) ((a < b) ? a : b)

#define OTHER_DRONES_ARRAY_SIZE 10  // size of the array containing the positions of other drones. must be at least as high as the highest id among all drones plus one.
#define DUMMY_POSITION 99999  // the xyz coordinates of non-connected drones will be set to this value

//#define DEBUG_MODULE "PUSH

typedef enum
{
  uninitialized,
  enginesOff,
  active,
  debug1,
  debug2,
  debug3,
} State;

typedef struct _PacketData
{
  int id;
  Vector3 pos;
} PacketData;  // size: ? + 12 bytes


static PacketData packetData;  // the data that is send around via the p2p broadcast method
static Vector3 targetPosition;  // this drones's target position
static Vector3 otherPositions[OTHER_DRONES_ARRAY_SIZE];  // array of the positions of the other drones
// static uint8_t lastReceivedDroneId;  // id of the last received packetData
static uint8_t droneAmount;  // amount of drones. SET DURING INITIALIZATION, DON'T CHANGE AT RUNTIME.
static uint8_t timer;
// variables for the avoidance algorithm, values are set from the pc. default values exist just in case something goes wrong.
static float forceFalloff = 1.5;
static float targetForce = 0.3;
static float avoidRange = 1.0;
static float avoidForce = 1.5;
static float maxLength = 1.0;


static void communicate()
{
  if (timer == 2 * packetData.id)
  {
    // consolePrintf("%d comm \n", packetData.id);
    P2PPacket packet;
    packet.port = 0;
    packet.size = sizeof(PacketData);
    memcpy(packet.data, &packetData, sizeof(PacketData));
    radiolinkSendP2PPacketBroadcast(&packet);
  }
}

void p2pCallbackHandler(P2PPacket *p)
{
  PacketData receivedPacketData;
  memcpy(&receivedPacketData, p->data, sizeof(PacketData));
  otherPositions[receivedPacketData.id] = receivedPacketData.pos;
  // consolePrintf("%d <- id=%d\n", packetData.id, receivedPacketData.id);
}

static Vector3 getTargetVector()
{
  Vector3 droneToTarget = sub(packetData.pos, targetPosition);
  if(magnitude(droneToTarget) > forceFalloff)
  {
    droneToTarget = norm(droneToTarget);
  }
  else
  {
    droneToTarget = mul(droneToTarget, 1 / forceFalloff);
  }
  droneToTarget = mul(droneToTarget, targetForce);
  return droneToTarget;
}

static Vector3 getAvoidVector(bool *isInAvoidRange)
{
  *isInAvoidRange = false;
  // other drones stuff (maybe quit early if otherPositions[i].x == DUMMY_POSITION?)
  Vector3 sum = (Vector3){0, 0, 0};
  for (int i = 0; i < OTHER_DRONES_ARRAY_SIZE; i++)
  {
    Vector3 otherToDrone = sub(otherPositions[i], packetData.pos);
    float distance = magnitude(otherToDrone);
    if(distance < avoidRange)
    {
      *isInAvoidRange = true;
      otherToDrone = norm(otherToDrone);
      otherToDrone = mul(otherToDrone, 1 - (distance / avoidRange));
      otherToDrone = mul(otherToDrone, avoidForce);
      sum = add(sum, otherToDrone);
    }
  }
  return sum;
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
  bool isInAvoidRange = false;  // true if the drone is close within avoidRange of another one
  bool isLanding = false;  // true if the drone is requested to land
  
  timer = 0;

  // drone.cmd value meanings:
  // 1:   start
  // 2:   land
  // 3:   debug1
  // 4:   debug2
  // 5:   idle
  // 6:   reset timer
  // 100: used to trigger initialization
  // be careful not to use these values for something else
  static int8_t droneCmd = 0;

  // parameters can be written from the pc and read by the drone
  PARAM_GROUP_START(drone)
  PARAM_ADD(PARAM_UINT8, amount, &droneAmount)
  PARAM_ADD(PARAM_UINT8, id, &packetData.id)
  PARAM_ADD(PARAM_INT8, cmd, &droneCmd)
  PARAM_ADD(PARAM_FLOAT, targetX, &targetPosition.x)
  PARAM_ADD(PARAM_FLOAT, targetY, &targetPosition.y)
  PARAM_ADD(PARAM_FLOAT, targetZ, &targetPosition.z)
  PARAM_ADD(PARAM_FLOAT, forceFalloff, &forceFalloff)
  PARAM_ADD(PARAM_FLOAT, targetForce, &targetForce)
  PARAM_ADD(PARAM_FLOAT, avoidRange, &avoidRange)
  PARAM_ADD(PARAM_FLOAT, avoidForce, &avoidForce)
  PARAM_ADD(PARAM_FLOAT, maxLength, &maxLength)
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

  // put PacketData structs with out of bounds values into the otherPositions array
  for (int i = 0; i < OTHER_DRONES_ARRAY_SIZE; i++)
  {
    otherPositions[i] = (Vector3){DUMMY_POSITION, DUMMY_POSITION, DUMMY_POSITION};
  }

  while (1)
  {
    // vTaskDelay(M2T(10));
    vTaskDelay(M2T(10));
    timer += 1;
    if(timer == 2 * 10)
    {
      timer = 0;
    }

    // don't execute the entire while loop before initialization happend
    if (state == uninitialized)
    {
      if (droneCmd == 100)
      {
        state = enginesOff;
        droneCmd = 0;
      }
      continue;
    }

    // put kalman position in this drone's packetData struct
    estimatorKalmanGetEstimatedPos(&kalmanPosition);
    packetData.pos.x = kalmanPosition.x;
    packetData.pos.y = kalmanPosition.y;
    packetData.pos.z = kalmanPosition.z;

    switch (droneCmd)
    {
      case 0:
        break;
      case 1:  // start
        targetPosition.x = packetData.pos.x;
        targetPosition.y = packetData.pos.y;
        targetPosition.z = 0.7f;
        isLanding = false;
        state = active;
        break;
      case 2:  // land
        targetPosition.x = packetData.pos.x;
        targetPosition.y = packetData.pos.y;
        targetPosition.z = -1.0f;
        isLanding = true;
        state = active;
        break;
      case 3:  // debug1
        state = debug1;
        consolePrintf("Drone %d entered debug1 state \n", packetData.id);
        break;
      case 4:  // debug2
        state = debug2;
        consolePrintf("Drone %d entered debug2 state \n", packetData.id);
        break;
      case 5:  // off
        state = enginesOff;
        consolePrintf("Drone %d entered enginesOff state \n", packetData.id);
        break;
      case 6:  // reset
        timer = 0;
        consolePrintf("Timer reset for drone %d \n", packetData.id);
        break;
      case 10:  // info
        consolePrintf("%d: target x=%.2f y=%.2f z=%.2f \n", packetData.id, (double)targetPosition.x, (double)targetPosition.y, (double)targetPosition.z);
        consolePrintf("%d: forceFalloff=%.2f targetForce=%.2f avoidRange=%.2f avoidForce=%.2f maxLength=%.2f \n", packetData.id, (double)forceFalloff, (double)targetForce, (double)avoidRange, (double)avoidForce, (double)maxLength);
        break;
      default:
        break;
    }
    droneCmd = 0;

    switch (state)
    {
      case active:
        communicate();
        Vector3 moveVector = getTargetVector();
        moveVector = add(moveVector, getAvoidVector(&isInAvoidRange));
        moveVector = clamp(moveVector, maxLength);
        moveVector = add(moveVector, packetData.pos);
        // consolePrintf("%d: x=%.2f y=%.2f z=%.2f \n", packetData.id, (double)moveVector.x, (double)moveVector.y, (double)moveVector.z);
        setHoverSetpoint(&setpoint, moveVector.x, moveVector.y, moveVector.z);
        if (isInAvoidRange)
        {
          ledSetAll();
        }
        else
        {
          ledClearAll();
        }
        if (isLanding && packetData.pos.z < 0.15f)
        {
          state = enginesOff;
          consolePrintf("Drone %d entered enginesOff state \n", packetData.id);
        }
        break;
      case enginesOff:
        communicate();
        getAvoidVector(&isInAvoidRange);
        if (isInAvoidRange)
        {
          ledSetAll();
        }
        else
        {
          ledClearAll();
        }
        shutOffEngines(&setpoint);
        break;
      case debug1:
        communicate();
        Vector3 avoidVector = getAvoidVector(&isInAvoidRange);
        if (packetData.id == 4)
        {
          consolePrintf("%d: x=%.2f y=%.2f z=%.2f \n", packetData.id, (double)avoidVector.x, (double)avoidVector.y, (double)avoidVector.z);
        }
        if (isInAvoidRange)
        {
          ledSetAll();
        }
        else
        {
          ledClearAll();
        }
        break;
      case debug2:
        consolePrintf("%d: x=%.2f y=%.2f z=%.2f \n", packetData.id, (double)packetData.pos.x, (double)packetData.pos.y, (double)packetData.pos.z);
        break;
      case uninitialized:  // this case should never occur since the drone should have been initialized before the switch statement
      default:
        ledClearAll();
        shutOffEngines(&setpoint);
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

// static void approachTargetAvoidOthers(setpoint_t *sp, bool *isInAvoidRange)
// {
//   Vector3 dronePosition = packetData.pos;
//   Vector3 sum = (Vector3){0, 0, 0};
//   *isInAvoidRange = false;
//   // target stuff
//   Vector3 droneToTarget = sub(dronePosition, targetPosition);
//   if(magnitude(droneToTarget) > forceFalloff)
//   {
//     droneToTarget = norm(droneToTarget);
//   }
//   else
//   {
//     droneToTarget = mul(droneToTarget, 1 / forceFalloff);
//   }
//   droneToTarget = mul(droneToTarget, targetForce);
//   sum = add(sum, droneToTarget);
//   // other drones stuff (maybe quit early if otherPositions[i].x == DUMMY_POSITION?)
//   for (int i = 0; i < OTHER_DRONES_ARRAY_SIZE; i++)
//   {
//     Vector3 otherToDrone = sub(otherPositions[i], dronePosition);
//     float distance = magnitude(otherToDrone);
//     if(distance < avoidRange)
//     {
//       *isInAvoidRange = true;
//       otherToDrone = norm(otherToDrone);
//       otherToDrone = mul(otherToDrone, 1 - (distance / avoidRange));
//       otherToDrone = mul(otherToDrone, avoidForce);
//       sum = add(sum, otherToDrone);
//     }
//   }
//   // add sum of forces to drone position and apply as setpoint
//   if (packetData.id == 4)
//   {
//     consolePrintf("%d: x=%.2f y=%.2f z=%.2f \n", packetData.id, (double)sum.x, (double)sum.y, (double)sum.z);
//   }
//   sum = add(dronePosition, sum);
//   setHoverSetpoint(sp, sum.x, sum.y, sum.z);
// }


      // case starting:
      //   moveVertical(&setpoint, 0.4);
      //   if (packetData.pos.z > 0.7f)
      //   {
      //     targetPosition.x = packetData.pos.x;
      //     targetPosition.y = packetData.pos.y;
      //     targetPosition.z = packetData.pos.z;
      //     consolePrintf("%d entered active state x=%.2f y=%.2f z=%.2f \n", packetData.id, (double)targetPosition.x, (double)targetPosition.y, (double)targetPosition.z);
      //     state = active;
      //   }
      //   break;
      // case landing:
      //   moveVertical(&setpoint, -0.4);
      //   if (packetData.pos.z < 0.05f)
      //   {
      //     state = enginesOff;
      //     consolePrintf("Drone %d entered enginesOff state \n", packetData.id);
      //   }
      //   break;

      
// static void moveVertical(setpoint_t *sp, float zVelocity)
// {
//   sp->mode.x = modeVelocity;
//   sp->mode.y = modeVelocity;
//   sp->mode.z = modeVelocity;
//   sp->mode.yaw = modeAbs;

//   sp->velocity.x = 0.0;
//   sp->velocity.y = 0.0;
//   sp->velocity.z = zVelocity;
//   sp->attitudeRate.yaw = 0.0;
// }