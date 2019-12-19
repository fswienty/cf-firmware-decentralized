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

#define OTHERDRONESAMOUNT 10  // size of the array containing the positions of other drones. must be at least as high as the highest id among all drones plus one.
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
  debug3,
} State;

typedef struct _PacketData
{
  int32_t ctr;
  uint8_t id;
  Vector3 pos;
} PacketData;  // size: ? + 12 bytes


static PacketData packetData;  // oi
// static Vector3 positition;  // this drone's position
static Vector3 targetPosition;  // this drones's target position
static Vector3 otherPositions[OTHERDRONESAMOUNT];  // array of the positions of the other drones
// static uint8_t expectedCtr;  // the ctr the next packet should have
static uint8_t lastReceivedDroneId;  // id of the last received packetData
static uint8_t prevDroneId;  // Id of the previous drone in the cycle. SET DURING INITIALIZATION, DON'T CHANGE AT RUNTIME.
static uint8_t nextDroneId;  // Id of the next drone in the cycle. SET DURING INITIALIZATION, DON'T CHANGE AT RUNTIME.
static uint8_t droneAmount;  // amount of drones. SET DURING INITIALIZATION, DON'T CHANGE AT RUNTIME.

static bool mustSend;
// static bool commContinued;

static void communicate()
{
  if (mustSend)
  {
    consolePrintf("%d comm \n", packetData.id);

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

  consolePrintf("%d rec pkt: id=%d ctr=%d \n", packetData.id, receivedPacketData.id, receivedPacketData.ctr);
  // otherPositions[receivedPacketData.id] = receivedPacketData.pos;


  if(receivedPacketData.ctr != packetData.ctr - 1 + droneAmount)
  {
    mustSend = false;
  }

  if (receivedPacketData.ctr == packetData.ctr - 1 + droneAmount)
  {
    mustSend = true;
    packetData.ctr = receivedPacketData.ctr + 1;
  }

  // consolePrintf("%d got: id=%d, x=%.2f \n", packetData.id, receivedPacketData.id, receivedPacketData.pos.x);
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

// static bool checkDistances()
// {
//   consolePrintf("Drone %d checks its distances", packetData.id);
//   Vector3 dronePosition = packetData.pos;
//   bool otherIsClose = false;
//   for (int i = 0; i < OTHERDRONESAMOUNT; i++)
//   {
//     Vector3 otherPosition = otherPositions[i].pos;
//     if (otherPosition.x == DUMMYPOSITION)
//     {
//       continue;
//     }
//     Vector3 droneToOther = sub(dronePosition, otherPosition);
//     float distance = magnitude(droneToOther);
//     if (distance < 0.3f)
//     {
//       otherIsClose = true;
//     }
//   }
//   return otherIsClose;
// }

static void approachTarget(setpoint_t *sp)
{
  Vector3 dronePosition = packetData.pos;
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

  // drone.cmd value meanings:
  // 100: used to trigger initialization
  // 1:  start
  // 2:  land
  // 3:  debug1
  // 4:  debug2
  // 5:  trigger a drone to start the communication
  // be careful not to use these values for something else
  static int8_t droneCmd = 0;
  PARAM_GROUP_START(drone)
  PARAM_ADD(PARAM_UINT8, amount, &droneAmount)
  PARAM_ADD(PARAM_UINT8, id, &packetData.id)
  PARAM_ADD(PARAM_UINT8, prevId, &prevDroneId)
  PARAM_ADD(PARAM_UINT8, nextId, &nextDroneId)
  PARAM_ADD(PARAM_INT32, ctr, &packetData.ctr)
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
    // vTaskDelay(M2T(10));
    vTaskDelay(M2T(2000));
    //consolePrintf("int %d, string %s", 546, "benis");

    // don't execute the entire while loop before initialization happend
    if (state == uninitialized)
    {
      if (droneCmd == 100)
      {
        // INIT
        // lastReceivedDroneId = 255;

        // put PacketData structs with out of bounds values into the otherPositions array
        for (int i = 0; i < OTHERDRONESAMOUNT; i++)
        {
          // PacketData dummy;
          // dummy.id = i;
          // dummy.pos.x = DUMMYPOSITION;
          // dummy.pos.y = DUMMYPOSITION;
          // dummy.pos.z = DUMMYPOSITION;
          otherPositions[i] = (Vector3){DUMMYPOSITION, DUMMYPOSITION, DUMMYPOSITION};
        }
        dbgchr = packetData.id;
        state = enginesOff;
        droneCmd = 0;
      }
      continue;
    }

    // put kalman position in this drone's packetData struct
    estimatorKalmanGetEstimatedPos(&kalmanPosition);
    // packetData.pos.x = kalmanPosition.x;
    // packetData.pos.y = kalmanPosition.y;
    // packetData.pos.z = kalmanPosition.z;
    // packetData.pos.x = targetPosition.x;
    // packetData.pos.y = targetPosition.y;
    // packetData.pos.z = targetPosition.z;
    // positition.x = targetPosition.x;
    // positition.y = targetPosition.y;
    // positition.z = targetPosition.z;

    switch (droneCmd)
    {
      case 1:
        state = starting;
        break;
      case 2:
        state = landing;
        break;
      case 3:  // comm
        state = debug1;
        consolePrintf("Drone %d entered communication debug state\n", packetData.id);
        break;
      case 4:
        state = enginesOff;
        consolePrintf("Drone %d entered idle state\n", packetData.id);
        break;
      case 6:
        communicate();
        consolePrintf("Drone %d executed communicate() once\n", packetData.id);
        break;
      case 5: // trigger
        // lastReceivedDroneId = prevDroneId;
        packetData.ctr = droneAmount;
        mustSend = true;
        break;
      case 10: // debug
        consolePrintf("%s\n", "##############");
        consolePrintf("Drone %d info:\n", packetData.id);
        consolePrintf("pos x: %.2f y: %.2f z: %.2f\n", packetData.pos.x, packetData.pos.y, packetData.pos.z);
        consolePrintf("target x: %.2f y: %.2f z: %.2f\n", targetPosition.x, targetPosition.y, targetPosition.z);
        // consolePrintf("triggerId: %d\n", prevDroneId);
        // consolePrintf("receivedId: %d\n", lastReceivedDroneId);
        consolePrintf("%s\n", "##############");
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
        if (packetData.pos.z > 0.7f)
        {
          targetPosition.x = packetData.pos.x;
          targetPosition.y = packetData.pos.y;
          targetPosition.z = packetData.pos.z;
          state = flying;
        }
        break;
      case landing:
        moveVertical(&setpoint, -0.4);
        if (packetData.pos.z < 0.15f)
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
        // if (checkDistances())
        // {
        //   ledSetAll();
        // }
        // else
        // {
        //   ledClearAll();
        // }
        break;
      case debug2:
        break;
      case debug3:
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
