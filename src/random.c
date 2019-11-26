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

#define DEBUG_MODULE "PUSH"

static uint8_t port = 0;
static uint8_t size = 0;
static uint8_t data = 0;
static uint8_t rssi = 0;

void p2pcallbackHandler(P2PPacket *p)
{
  port = p->port;
  size = p->size;
  data = p->data[0];
  rssi = p->rssi;
}

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

static void use_float(float var)
{
  return;
}
static void use_int(int var)
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

  static P2PPacket pk;
  pk.port = 0;
  pk.size = 4;

  static float posX = 0;
  static float posY = 0;
  static float posZ = 0;

  // vTaskDelay(M2T(500)); // wait x ms, M2T: ms to os ticks
  // resetEstimator();
  // vTaskDelay(M2T(500));

  //estimatorKalmanGetEstimatedPos(&kalmanPosition);
  //uint16_t idUp = logGetVarId("range", "up");

  DEBUG_PRINT("Waiting for activation ...\n");

  static int8_t cmd = 0;
  PARAM_GROUP_START(cmd)
  PARAM_ADD(PARAM_INT8, cmd, &cmd)
  PARAM_GROUP_STOP(cmd)

  static int8_t send = 0;
  PARAM_GROUP_START(p2p)
  PARAM_ADD(PARAM_UINT8, send, &send)
  PARAM_GROUP_STOP(p2p)


  LOG_GROUP_START(p2p)
  LOG_ADD(LOG_UINT8, send, &send)
  LOG_ADD(LOG_UINT8, port, &port)
  LOG_ADD(LOG_UINT8, size, &size)
  LOG_ADD(LOG_UINT8, data, &data)
  LOG_ADD(LOG_UINT8, rssi, &rssi)
  LOG_GROUP_STOP(p2p)

  static int8_t myNumber = 0;
  LOG_GROUP_START(ro)
  LOG_ADD(LOG_INT8, size, &myNumber)
  LOG_GROUP_STOP(ro)

  static float rwTest = 0;
  PARAM_GROUP_START(rw)
  PARAM_ADD(PARAM_FLOAT, test, &rwTest)
  PARAM_GROUP_STOP(rw)
  LOG_GROUP_START(rw)
  LOG_ADD(LOG_FLOAT, test, &rwTest)
  LOG_GROUP_STOP(rw)


  p2pRegisterCB(p2pcallbackHandler);

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

    switch (cmd)
    {
    case 0:
      break;
    case 1:
      ledClearAll();
      break;
    case 2:
      ledSetAll();
      break;
    case 3:
      ledSet(LED_BLUE_L, true);
      break;
    case 4:
      ledSet(LED_GREEN_L, true);
      break;
    case 5:
      ledSet(LED_GREEN_R, true);
      float benis = 443.653;
      rwTest = (float)sizeof(benis);
      break;
    case 6:
      ledSet(LED_RED_L, true);
      rwTest = (float)sizeof("Hello World");
      break;
    case 100:
      memcpy(pk.data, &send, 4);
      radiolinkSendP2PPacketBroadcast(&pk);
      break;
    default:
      break;
    }
    cmd = 0;


    use_float(posX + posY + posZ);
    // use_int(myNumber);
    // use_int(command);
    use_int(4);

    if (1)
    {
      setHoverSetpoint(&setpoint, 0, 0, .3, 0);
    }
  }
}

void waitForPositionEstimator()
{
}
