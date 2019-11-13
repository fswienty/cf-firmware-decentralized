#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app.h"

#include "commander.h"

#include "FreeRTOS.h"
#include "task.h"

#include "debug.h"

#include "log.h"

// my stuff
#include "estimator_kalman.h"


#define DEBUG_MODULE "PUSH"

static void setHoverSetpoint(setpoint_t *setpoint, float x, float y, float z, float yaw)
{
  setpoint->mode.x = modeAbs;
  setpoint->mode.y = modeAbs;
  setpoint->mode.z = modeAbs;

  setpoint->position.x = x;
  setpoint->position.y = y;
  setpoint->position.z = z;

  setpoint->mode.yaw = modeAbs;
  setpoint->attitude.yaw = yaw;

  //setpoint->velocity_body = true;
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
  static point_t kalmanPosition;

  vTaskDelay(M2T(3000)); // wait x ms, M2T: ms to os ticks
  resetEstimator();
  vTaskDelay(M2T(10000));

  estimatorKalmanGetEstimatedPos(&kalmanPosition);
  //uint16_t idUp = logGetVarId("range", "up");

  DEBUG_PRINT("Waiting for activation ...\n");

  while (1)
  {
    vTaskDelay(M2T(10));
    if (1)
    {
      setHoverSetpoint(&setpoint, 0, 0, .3, 0);
      commanderSetSetpoint(&setpoint, 3);
    }
  }
}

void waitForPositionEstimator()
{

}
