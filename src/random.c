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

#define DEBUG_MODULE "PUSH"

static void setHoverSetpoint(setpoint_t *setpoint, float x, float y, float z, float yaw)
{
  setpoint->mode.x = modeAbs;
  setpoint->mode.y = modeAbs;
  setpoint->mode.z = modeAbs;
  setpoint->mode.yaw = modeAbs;

  setpoint->position.x = x;
  setpoint->position.y = y;
  setpoint->position.z = z;
  setpoint->attitude.yaw = yaw;
}

static void use(float var)
{
  return;
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
  static setpoint_t setpoint;
  //static point_t kalmanPosition;
  static int varid;

  static float myNumber = 0;
  
  /*    change this number via the crazyradio to make the crazyflie do stuff
  *     0: nothing
  *     1: start
  *    -1: land
  */
  static int8_t command = 0;

  static float posX = 0;
  static float posY = 0;
  static float posZ = 0;

  // vTaskDelay(M2T(500)); // wait x ms, M2T: ms to os ticks
  // resetEstimator();
  // vTaskDelay(M2T(500));

  //estimatorKalmanGetEstimatedPos(&kalmanPosition);
  //uint16_t idUp = logGetVarId("range", "up");

  DEBUG_PRINT("Waiting for activation ...\n");

  LOG_GROUP_START(test)
  LOG_ADD(LOG_FLOAT, myNumber, &myNumber)
  LOG_GROUP_STOP(test)
  myNumber = 0;

  PARAM_GROUP_START(command)
  PARAM_ADD(PARAM_INT8, command, &command)
  PARAM_GROUP_STOP(command)
  command = 42;

  while (1)
  {
    vTaskDelay(M2T(10));

    // get current position
    varid = logGetVarId("kalman", "stateX");
    posX = logGetFloat(varid);
    varid = logGetVarId("kalman", "stateY");
    posY = logGetFloat(varid);
    varid = logGetVarId("kalman", "stateZ");
    posZ = logGetFloat(varid);

    switch (command)
    {
    case 0:
      myNumber += 0;
      break;
    case 1:
      myNumber += 1337;
      break;
    case -1:
      myNumber -= 1337;
      break;
    default:
      break;
    }
    //command = 0;
    use(posX + posY + posZ);

    if (1)
    {
      setHoverSetpoint(&setpoint, 0, 0, .3, 0);
    }
  }
}

void waitForPositionEstimator()
{
}
