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

typedef struct _PositionP2P
{
  char id;
  float x;
  float y;
  float z;
} PositionP2P; // size: 4 + 12 bytes

float sentFloat;
float receivedFloat;

static PositionP2P receivedPositionP2P;

// void p2pcallbackHandler(P2PPacket *p)
// {
//   port = p->port;
//   size = p->size;
//   for (int i = 0; i < 10; i++)
//   {
//     dataArray[i] = p->data[i];
//   }
//   rssi = p->rssi;
// }

void p2pcallbackHandler2(P2PPacket *p)
{
  // put the received positionp2p struct into the array at the position corresponding to the received id
  //memcpy(&receivedPositionP2P, &(p->data), sizeof(PositionP2P));
  memcpy(&receivedFloat, &(p->data), sizeof(float));
  //positionP2PArray[receivedPos.id] = receivedPos;
}

// void init()
// {
//   // put PositionP2P structs with out of bounds values into the positionP2PArray
//   int size = sizeof(positionP2PArray) / sizeof(positionP2PArray[0]);
//   for (int i = 0; i < size; i++)
//   {
//     PositionP2P dummy = {.id = i, .x = 9999, .y = 9999, .z = 9999};
//     positionP2PArray[i] = dummy;
//   }
// }

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
  static int8_t id = 0;
  static int8_t amount = 0;
  PARAM_GROUP_START(drone)
  PARAM_ADD(PARAM_UINT8, amount, &amount)
  PARAM_ADD(PARAM_UINT8, id, &id)
  PARAM_GROUP_STOP(drone)

  static int8_t cmd = 0;
  static int8_t cmdData = 0;
  PARAM_GROUP_START(cmd)
  PARAM_ADD(PARAM_INT8, cmd, &cmd)
  PARAM_ADD(PARAM_INT8, data, &cmdData)
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


  static float dbgflt = 0;
  static char dbgchr = 0;
  PARAM_GROUP_START(dbg)
  PARAM_ADD(PARAM_FLOAT, flt, &dbgflt)
  PARAM_ADD(PARAM_UINT8, chr, &dbgchr)
  PARAM_GROUP_STOP(dbg)
  LOG_GROUP_START(dbg)
  LOG_ADD(LOG_FLOAT, flt, &dbgflt)
  PARAM_ADD(LOG_UINT8, chr, &dbgchr)
  LOG_GROUP_STOP(dbg)


  static setpoint_t setpoint;
  //static point_t kalmanPosition;
  static int varid;

  static PositionP2P positionP2P;
  positionP2P.id = 2;
  positionP2P.x = 2.5;
  positionP2P.y = -1;
  positionP2P.z = 1;

  sentFloat = 46.52;


  static P2PPacket pk;
  pk.port = 0;


  static float posX = 0;
  static float posY = 0;
  static float posZ = 0;

  // vTaskDelay(M2T(500)); // wait x ms, M2T: ms to os ticks
  // resetEstimator();
  // vTaskDelay(M2T(500));

  //estimatorKalmanGetEstimatedPos(&kalmanPosition);
  //uint16_t idUp = logGetVarId("range", "up");

  DEBUG_PRINT("Waiting for activation ...\n");

  p2pRegisterCB(p2pcallbackHandler2);
  //init();

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

    dbgchr = pk.data[cmdData];

    switch (cmd)
    {
      case 1:
        dbgchr = receivedPositionP2P.id;
        break;
      case 2:
        dbgflt = receivedPositionP2P.x;
        break;
      case 3:
        dbgflt = receivedPositionP2P.y;
        break;
      case 4:
        dbgflt = receivedPositionP2P.z;
        break;
      case 5:
        dbgflt = receivedFloat;
        break;
      case 100:
        pk.size = 11;
        memcpy(pk.data, "Hello World", 11);
        radiolinkSendP2PPacketBroadcast(&pk);
        break;
      case 101:
        // dbgflt = sizeof(PositionP2P);
        pk.size = sizeof(PositionP2P);
        memcpy(pk.data, &positionP2P, sizeof(PositionP2P));
        radiolinkSendP2PPacketBroadcast(&pk);
        memcpy(&receivedPositionP2P, pk.data, sizeof(PositionP2P));
        break;
      case 102:
        pk.size = sizeof(float);
        memcpy(&pk.data, &sentFloat, sizeof(float));
        radiolinkSendP2PPacketBroadcast(&pk);
        memcpy(&receivedFloat, &pk.data, sizeof(float));
        break;
      default:
        break;
    }
    cmd = 0;

    if (1)
    {
      setHoverSetpoint(&setpoint, 0, 0, .3, 0);
    }
  }
}

void waitForPositionEstimator()
{
}

// ledClearAll();
// ledSetAll();
// ledSet(LED_BLUE_L, true);
