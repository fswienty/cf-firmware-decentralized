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

#define OTHER_DRONES_ARRAY_SIZE 10  // size of the array containing the positions of other drones. must be at least as high as the highest id among all drones plus one.
#define DUMMY_VALUE 99999  // the xyz coordinates and velocities of non-connected drones will be set to this value

typedef enum
{
  uninitialized,
  enginesOff,
  simpleAvoid,
  flock,
  debug1,
  debug2,
  debug3,
} State;

typedef struct _PacketData
{
  int id;
  Vector3 pos;
  Vector3 vel;
} PacketData;  // size: ? bytes + 12 bytes + 12 bytes  // max: 60 bytes


static PacketData packetData;  // the data that is send around via the p2p broadcast method
static Vector3 targetPosition;  // this drones's target position
static Vector3 lastPosition;  // this drones position in the last execution cycle, used to estimate velocity
static Vector3 otherPositions[OTHER_DRONES_ARRAY_SIZE];  // array of the positions of the other drones
static Vector3 otherVelocities[OTHER_DRONES_ARRAY_SIZE];  // array of the velocities of the other drones
static uint8_t droneAmount;  // amount of drones. SET DURING INITIALIZATION, DON'T CHANGE AT RUNTIME.
static uint8_t timer;
// variables for the basic avoidance algorithm, values are set from the pc. default values exist just in case something goes wrong.
static float forceFalloff = 1.5;
static float targetForce = 0.3;
static float avoidRange = 1.0;
static float avoidForce = 1.5;
static float maxLength = 1.0;
// variables for the boid flocking algorithm
static float accBudget = 1.0;

static float wallMargin = 0.5;
static float xMax = 2 - wallMargin;
static float yMax = 1.5 - wallMargin;
static float zMax = 1.2 - wallMargin;
static float zMiddle = 1.0;

static float wWallAvoid = 1.0;
static float wSeparation = 1.0;
static float sepRange = 1.0;
static float wAlignment = 1.0;
static float alignRange = 1.0;
static float wCohesion = 1.0;
static float cohesRange = 1.0;
static float wTargetSeek = 1.0;

#pragma region P2Pcomm
static void communicate()
{
  if (timer == packetData.id)
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
  otherVelocities[receivedPacketData.id] = receivedPacketData.vel;
  // consolePrintf("%d <- id=%d\n", packetData.id, receivedPacketData.id);
}
#pragma endregion P2Pcomm

#pragma region Flocking
static Vector3 getFlockVector(bool *isInAvoidRange)
{ 
  Vector3 flockVector = (Vector3){0, 0, 0};
  float remainingAcc = accBudget;

  // WALL AVOIDANCE
  Vector3 wallAvoidVector = (Vector3){0, 0, 0};
  float outsidednessX = 0;
  float outsidednessY = 0;
  if (packetData.pos.x > xMax) outsidednessX += abs(packetData.pos.x - xMax);
  if (packetData.pos.x < -xMax) outsidednessX += abs(packetData.pos.x + xMax);
  if (packetData.pos.y > yMax) outsidednessY += abs(packetData.pos.y - yMax);
  if (packetData.pos.y < -yMax) outsidednessY += abs(packetData.pos.y + yMax);
  // if (packetData.pos.z > (zMiddle + zMax)) outsidedness += abs(packetData.pos.z - (zMiddle + zMax));  // enable to make drones avoid the ceiling
  // if (packetData.pos.z < (zMiddle - zMax)) outsidedness += abs(packetData.pos.z - (zMiddle - zMax));  // enable to make drones avoid the floor

  float outsidedness = outsidednessX > outsidednessY ? outsidednessX : outsidednessY;

  if (outsidedness > 0)
  {
    wallAvoidVector = sub(packetData.pos, (Vector3){0, 0, zMiddle});  // maybe replace zMiddle with the current height of the drone?
    wallAvoidVector = norm(wallAvoidVector);
    wallAvoidVector = mul(wallAvoidVector, outsidedness);
  }
  addToFlockVector(&flockVector, &remainingAcc, wallAvoidVector, wWallAvoid);

  // SEPARATION
  Vector3 separationVector = (Vector3){0, 0, 0};
  *isInAvoidRange = false;
  for (int i = 0; i < OTHER_DRONES_ARRAY_SIZE; i++)
  {
    Vector3 otherToDrone = sub(otherPositions[i], packetData.pos);
    float distance = magnitude(otherToDrone);
    if(distance < sepRange)
    {
      *isInAvoidRange = true;
      otherToDrone = norm(otherToDrone);
      otherToDrone = mul(otherToDrone, 1 - (distance / sepRange));
      // otherToDrone = mul(otherToDrone, avoidForce);
      separationVector = add(separationVector, otherToDrone);
    }
  }
  addToFlockVector(&flockVector, &remainingAcc, separationVector, wSeparation);

  // ALIGNMENT
  Vector3 alignmentVector = packetData.vel;
  int dronesInAlignRange = 1;
  for (int i = 0; i < OTHER_DRONES_ARRAY_SIZE; i++)
  {
    Vector3 otherToDrone = sub(otherPositions[i], packetData.pos);
    float distance = magnitude(otherToDrone);
    if(distance < alignRange)
    {
      dronesInAlignRange += 1;
      alignmentVector = add(alignmentVector, otherVelocities[i]);
    }
  }
  alignmentVector = mul(alignmentVector, 1 / dronesInAlignRange);
  alignmentVector = clamp(alignmentVector, 0.2f);  // maybe make this configurable
  addToFlockVector(&flockVector, &remainingAcc, alignmentVector, wAlignment);

  // COHESION
  Vector3 cohesionVector = (Vector3){0, 0, 0};
  int dronesInCohesionRange = 0;
  for (int i = 0; i < OTHER_DRONES_ARRAY_SIZE; i++)
  {
    Vector3 otherToDrone = sub(otherPositions[i], packetData.pos);
    float distance = magnitude(otherToDrone);
    if(distance < cohesRange)
    {
      dronesInCohesionRange += 1;
      cohesionVector = add(cohesionVector, otherPositions[i]);
    }
  }
  if (dronesInCohesionRange > 0)
  {
    cohesionVector = mul(cohesionVector, 1 / dronesInCohesionRange);
    cohesionVector = clamp(cohesionVector, 0.2f);  // maybe make this configurable
  }
  addToFlockVector(&flockVector, &remainingAcc, cohesionVector, wCohesion);

  // TARGET SEEKING
  Vector3 targetVector = sub(packetData.pos, targetPosition);
  if(magnitude(targetVector) > forceFalloff)
  {
    targetVector = norm(targetVector);
  }
  else
  {
    targetVector = mul(targetVector, 1 / forceFalloff);
  }
  addToFlockVector(&flockVector, &remainingAcc, targetVector, wTargetSeek);

  return clamp(flockVector, maxLength);
}

void addToFlockVector(Vector3 *flockVector, float *remainingAcc, Vector3 vector, float weight)
{
  if (*remainingAcc < 0)
  {
    return;
  }
  Vector3 vec = mul(vector, weight);
  float length = magnitude(vec);
  if (*remainingAcc > length)
  {
    *flockVector = add(*flockVector, vec);
  }
  else
  {
    *flockVector = add(*flockVector, clamp(vec, *remainingAcc));
  }
  *remainingAcc -= length;
}
#pragma endregion Flocking

#pragma region SimpleAvoid
static Vector3 getSimpleAvoidVector(bool *isInAvoidRange)
{
  Vector3 vector = getTargetVector();
  vector = add(vector, getAvoidVector(isInAvoidRange));  // check if this works
  return clamp(vector, maxLength);
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
  // other drones stuff (maybe quit early if otherPositions[i].x == DUMMY_VALUE?)
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
#pragma endregion SimpleAvoid

#pragma region Setpoints_LED
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

static void ledIndicateDetection(bool isInRange)
{
  if (isInRange)
  {
    ledSetAll();
  }
  else
  {
    ledClearAll();
  }
}
#pragma endregion Setpoints_LED

// ENTRY POINT
void appMain()
{
  static point_t kalmanPosition;
  static point_t kalmanVelocity;
  static setpoint_t setpoint;
  static State state = uninitialized;
  bool isInAvoidRange = false;  // true if the drone is close within avoidRange of another one
  bool isLanding = false;  // true if the drone is requested to land
  
  timer = 0;

  #pragma region Param_Log
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
  // drone.mode meaning:
  // 0: simple potential field avoidance and target approach
  // 1: boid flocking
  static int8_t droneMode = 0;

  // parameters can be written from the pc and read by the drone
  PARAM_GROUP_START(drone)
  PARAM_ADD(PARAM_UINT8, amount, &droneAmount)
  PARAM_ADD(PARAM_UINT8, id, &packetData.id)
  PARAM_ADD(PARAM_INT8, cmd, &droneCmd)
  PARAM_ADD(PARAM_INT8, mode, &droneCmd)
  PARAM_ADD(PARAM_FLOAT, targetX, &targetPosition.x)
  PARAM_ADD(PARAM_FLOAT, targetY, &targetPosition.y)
  PARAM_ADD(PARAM_FLOAT, targetZ, &targetPosition.z)
  PARAM_ADD(PARAM_FLOAT, forceFalloff, &forceFalloff)
  PARAM_ADD(PARAM_FLOAT, targetForce, &targetForce)
  PARAM_ADD(PARAM_FLOAT, avoidRange, &avoidRange)
  PARAM_ADD(PARAM_FLOAT, avoidForce, &avoidForce)
  PARAM_ADD(PARAM_FLOAT, maxLength, &maxLength)
  PARAM_ADD(PARAM_FLOAT, accBudget, &accBudget)
  PARAM_ADD(PARAM_FLOAT, zMiddle, &zMiddle)
  PARAM_ADD(PARAM_FLOAT, xMax, &xMax)
  PARAM_ADD(PARAM_FLOAT, yMax, &yMax)
  PARAM_ADD(PARAM_FLOAT, zMax, &zMax)
  PARAM_ADD(PARAM_FLOAT, wWallAvoid, &wWallAvoid)
  PARAM_ADD(PARAM_FLOAT, wSeparation, &wSeparation)
  PARAM_ADD(PARAM_FLOAT, sepRange, &sepRange)
  PARAM_ADD(PARAM_FLOAT, wAlignment, &wAlignment)
  PARAM_ADD(PARAM_FLOAT, alignRange, &alignRange)
  PARAM_ADD(PARAM_FLOAT, wCohesion, &wCohesion)
  PARAM_ADD(PARAM_FLOAT, cohesRange, &cohesRange)
  PARAM_ADD(PARAM_FLOAT, wTargetSeek, &wTargetSeek)
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
  #pragma endregion Param_Log

  p2pRegisterCB(p2pCallbackHandler);

  // put PacketData structs with out of bounds values into the otherPositions array
  for (int i = 0; i < OTHER_DRONES_ARRAY_SIZE; i++)
  {
    otherPositions[i] = (Vector3){DUMMY_VALUE, DUMMY_VALUE, DUMMY_VALUE};
    otherVelocities[i] = (Vector3){DUMMY_VALUE, DUMMY_VALUE, DUMMY_VALUE};
  }

  // MAIN LOOP
  while (1)
  {
    // vTaskDelay(M2T(10));
    vTaskDelay(M2T(10));
    timer += 1;
    if(timer == 10)
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

    estimatorKalmanGetEstimatedPos(&kalmanPosition);
    // put kalman position in this drone's packetData struct
    packetData.pos.x = kalmanPosition.x;
    packetData.pos.y = kalmanPosition.y;
    packetData.pos.z = kalmanPosition.z;
    // estimate velocity and put it in this drone's packetData struct
    packetData.vel.x = kalmanPosition.x - lastPosition.x;
    packetData.vel.y = kalmanPosition.y - lastPosition.y;
    packetData.vel.z = kalmanPosition.z - lastPosition.z;
    // update lastPosition for the next execution cycle
    lastPosition.x = kalmanPosition.x;
    lastPosition.y = kalmanPosition.y;
    lastPosition.z = kalmanPosition.z;

    // read droneCmd set from the ground station to handle commands
    switch (droneCmd)
    {
      case 0:
        break;
      case 1:  // start
        targetPosition.x = packetData.pos.x;
        targetPosition.y = packetData.pos.y;
        targetPosition.z = 0.7f;
        isLanding = false;
        if (droneMode == 0)
        {
          state = simpleAvoid;
        }
        if (droneMode == 1)
        {
          state = flock;
        }
        break;
      case 2:  // land
        targetPosition.x = packetData.pos.x;
        targetPosition.y = packetData.pos.y;
        targetPosition.z = -1.0f;
        isLanding = true;
        if (droneMode == 0)
        {
          state = simpleAvoid;
        }
        if (droneMode == 1)
        {
          state = flock;
        }
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

    // state machine for the quadcopter, executes the appropriate code depending on the current state
    // states might be changed by the ground station (see above) or from another state when some conditions are met
    switch (state)
    {
      case simpleAvoid:
        communicate();
        // Vector3 moveVector = getTargetVector();
        // moveVector = add(moveVector, getAvoidVector(&isInAvoidRange));
        // moveVector = clamp(moveVector, maxLength);
        // moveVector = add(moveVector, packetData.pos);
        Vector3 moveVector = add(packetData.pos, getSimpleAvoidVector(&isInAvoidRange));
        // consolePrintf("%d: x=%.2f y=%.2f z=%.2f \n", packetData.id, (double)moveVector.x, (double)moveVector.y, (double)moveVector.z);
        setHoverSetpoint(&setpoint, moveVector.x, moveVector.y, moveVector.z);

        ledIndicateDetection(isInAvoidRange);
        if (isLanding && packetData.pos.z < 0.15f)
        {
          state = enginesOff;
          consolePrintf("Drone %d entered enginesOff state \n", packetData.id);
        }
        break;
      case flock:
        communicate();
        Vector3 moveVector = add(packetData.pos, getFlockVector(&isInAvoidRange));
        setHoverSetpoint(&setpoint, moveVector.x, moveVector.y, moveVector.z);

        ledIndicateDetection(isInAvoidRange);
        if (isLanding && packetData.pos.z < 0.15f)
        {
          state = enginesOff;
          consolePrintf("Drone %d entered enginesOff state \n", packetData.id);
        }
        break;
      case enginesOff:
        communicate();
        getAvoidVector(&isInAvoidRange);
        shutOffEngines(&setpoint);
        ledIndicateDetection(isInAvoidRange);
        break;
      case debug1:
        communicate();
        Vector3 avoidVector = getAvoidVector(&isInAvoidRange);
        if (packetData.id == 4)
        {
          consolePrintf("%d: x=%.2f y=%.2f z=%.2f \n", packetData.id, (double)avoidVector.x, (double)avoidVector.y, (double)avoidVector.z);
        }
        ledIndicateDetection(isInAvoidRange);
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
