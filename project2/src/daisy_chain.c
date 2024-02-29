/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2019 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * push.c - App layer application of the onboard push demo. The crazyflie 
 * has to have the multiranger and the flowdeck version 2.
 */

#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "app.h"

#include "commander.h"

#include "FreeRTOS.h"
#include "task.h"

#include "radiolink.h"
#include "configblock.h"

#include "debug.h"

#include "log.h"
#include "param.h"

#include "sensorReading.h"

#define DEBUG_MODULE "PUSH"

#define BYTES_PER_PACKET 11
#define BYTES_PER_SECOND 100
#define DELAY_MS 1000*BYTES_PER_PACKET/BYTES_PER_SECOND


uint16_t other_left = (uint16_t)1000;
uint16_t other_right = (uint16_t)1000;
uint16_t other_front = (uint16_t)1000;
uint16_t other_back = (uint16_t)1000;
uint16_t other_up = (uint16_t)1000;

void p2pcallbackHandler(P2PPacket *p)
{
    /*
    data[0] - id
    data[1,2] - up
    data[3,4] - front
    data[5,6] - back
    data[7,8] - left
    data[9,10] - right
    */


  // Parse the data from the other crazyflie and print it
  uint8_t other_id = p->data[0];
  uint8_t rssi = p->rssi;
  memcpy(&other_up, &p->data[1], sizeof(uint16_t));
  memcpy(&other_front, &p->data[3], sizeof(uint16_t));
  memcpy(&other_back, &p->data[5], sizeof(uint16_t));
  memcpy(&other_left, &p->data[7], sizeof(uint16_t));
  memcpy(&other_right, &p->data[9], sizeof(uint16_t)); //11 bytes in total

  /*other_up = p->data[1];
  other_front = p->data[3];
  other_back = p->data[5];
  other_left = p->data[7];
  other_right = p->data[9];*/
  DEBUG_PRINT("\nrssi: %d ", rssi);
  DEBUG_PRINT("CF num. %d, the up dist is: %d\n", other_id, other_up);
}



static void setHoverSetpoint(setpoint_t *setpoint, float vx, float vy, float z, float yawrate)
{
  setpoint->mode.z = modeAbs;
  setpoint->position.z = z;


  setpoint->mode.yaw = modeVelocity;
  setpoint->attitudeRate.yaw = yawrate;


  setpoint->mode.x = modeVelocity;
  setpoint->mode.y = modeVelocity;
  setpoint->velocity.x = vx;
  setpoint->velocity.y = vy;

  setpoint->velocity_body = true;
}

typedef enum {
    idle,
    lowUnlock,
    unlocked,
    stopping
} State;

static State state = idle;

static const uint16_t unlockThLow = 100;
static const uint16_t unlockThHigh = 300;
static const uint16_t stoppedTh = 500;

static const float velMax = 1.0f;
static const uint16_t radius = 300;
static const uint16_t radius_up_down = 100;
static const float up_down_delta = 0.002f;

static float height_sp = 0.4f;

#define MAX(a,b) ((a>b)?a:b)
#define MIN(a,b) ((a<b)?a:b)

void appMain()
{
  static setpoint_t setpoint;

  p2pRegisterCB(p2pcallbackHandler);

  vTaskDelay(M2T(1000));

  logVarId_t idUp = logGetVarId("range", "up");
  logVarId_t idLeft = logGetVarId("range", "left");
  logVarId_t idRight = logGetVarId("range", "right");
  logVarId_t idFront = logGetVarId("range", "front");
  logVarId_t idBack = logGetVarId("range", "back");

  paramVarId_t idPositioningDeck = paramGetVarId("deck", "bcFlow2");
  paramVarId_t idMultiranger = paramGetVarId("deck", "bcMultiranger");

  sensorInit();

  float factor = velMax/radius;

  //DEBUG_PRINT("%i", idUp);

  DEBUG_PRINT("Waiting for activation ...\n");

  while(1) {
    vTaskDelay(M2T(DELAY_MS));
    //DEBUG_PRINT(".");

    uint8_t positioningInit = paramGetUint(idPositioningDeck);
    uint8_t multirangerInit = paramGetUint(idMultiranger);

    uint16_t my_up = logGetUint(idUp);
    uint16_t my_left = logGetUint(idLeft);
    uint16_t my_right = logGetUint(idRight);
    uint16_t my_front = logGetUint(idFront);
    uint16_t my_back = logGetUint(idBack);
    //DEBUG_PRINT("\nmy up dist is: %d",my_up);
    //sending packets:
    static P2PPacket p_reply;
    p_reply.port=0x00;
    p_reply.size=5*sizeof(uint16_t)+1*sizeof(uint8_t);
    uint64_t address = configblockGetRadioAddress();
    uint8_t my_id =(uint8_t)((address) & 0x00000000ff);

    memcpy(&p_reply.data[0], &my_id, sizeof(uint8_t));
    memcpy(&p_reply.data[1], &my_up, sizeof(uint16_t));
    memcpy(&p_reply.data[3], &my_front, sizeof(uint16_t));
    memcpy(&p_reply.data[5], &my_back, sizeof(uint16_t));
    memcpy(&p_reply.data[7], &my_left, sizeof(uint16_t));
    memcpy(&p_reply.data[9], &my_right, sizeof(uint16_t));


    radiolinkSendP2PPacketBroadcast(&p_reply);

    uint16_t up = (uint16_t)MIN(my_up,other_up);



    if (state == unlocked) {

        uint16_t front = (uint16_t)MIN(my_front,other_front);
        uint16_t back = (uint16_t)MIN(my_back,other_back);
        uint16_t left = (uint16_t)MIN(my_left,other_left);
        uint16_t right = (uint16_t)MIN(my_right,other_right);

      uint16_t left_o = radius - MIN(left, radius);
      uint16_t right_o = radius - MIN(right, radius);
      float l_comp = (-1) * left_o * factor;
      float r_comp = right_o * factor;
      float velSide = r_comp + l_comp;

      uint16_t front_o = radius - MIN(front, radius);
      uint16_t back_o = radius - MIN(back, radius);
      float f_comp = (-1) * front_o * factor;
      float b_comp = back_o * factor;
      float velFront = b_comp + f_comp;

      // we want to go up when there are obstacles (hands) closer than radius_up_down on both sides
      if(left < radius_up_down && right < radius_up_down)
      {
        height_sp += up_down_delta;
      }

      // we want to go down when there are obstacles (hands) closer than radius_up_down in front and back (or there is something on top)
      if((front < radius_up_down && back < radius_up_down) || up < radius)
      {
        height_sp -= up_down_delta;
      }

      uint16_t up_o = radius - MIN(up, radius);
      float height = height_sp - up_o/1000.0f;


      /*DEBUG_PRINT("l=%i, r=%i, lo=%f, ro=%f, vel=%f\n", left_o, right_o, l_comp, r_comp, velSide);
      DEBUG_PRINT("f=%i, b=%i, fo=%f, bo=%f, vel=%f\n", front_o, back_o, f_comp, b_comp, velFront);
      DEBUG_PRINT("u=%i, d=%i, height=%f\n", up_o, height);*/

      if (1) {
        setHoverSetpoint(&setpoint, velFront, velSide, height, 0);
        commanderSetSetpoint(&setpoint, 3);
      }

      if (height < 0.05f) {
        state = stopping;
        DEBUG_PRINT("X\n");
      }

    } else {

      if (state == stopping && up > stoppedTh) {
        DEBUG_PRINT("%i", up);
        state = idle;
        DEBUG_PRINT("S\n");
      }

      if (up < unlockThLow && state == idle && up > 0.001f) {
        DEBUG_PRINT("Waiting for hand to be removed!\n");
        state = lowUnlock;
      }

      if (up > unlockThHigh && state == lowUnlock && positioningInit && multirangerInit) {
        DEBUG_PRINT("Unlocked!\n");
        state = unlocked;
      }

      if (state == idle || state == stopping) {
        memset(&setpoint, 0, sizeof(setpoint_t));
        commanderSetSetpoint(&setpoint, 3);
      }
    }
  }
}


LOG_GROUP_START(other_cf)
  LOG_ADD(LOG_UINT16, up, &other_up)
  LOG_ADD(LOG_UINT16, front, &other_front)
  LOG_ADD(LOG_UINT16, back, &other_back)
  LOG_ADD(LOG_UINT16, left, &other_left)
  LOG_ADD(LOG_UINT16, right, &other_right)
LOG_GROUP_STOP(other_cf)

LOG_GROUP_START(Sensors)
  LOG_ADD(LOG_FLOAT, temp, &temperature)
  LOG_ADD(LOG_FLOAT, humid, &humidity)
LOG_GROUP_STOP(Sensors)
