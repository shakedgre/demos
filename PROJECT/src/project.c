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
#include <math.h>

#include "app.h"
#include "commander.h"
#include "crtp_commander_high_level.h"

#include "MovedecisionMaker.h"
#include "CONSTS.h"

#include "FreeRTOS.h"
#include "task.h"

#include "radiolink.h"
#include "configblock.h"

#include "debug.h"

#include "log.h"
#include "param.h"

#include "sensorReading.h"

#define DEBUG_MODULE "PUSH"

#define MAX(a,b) ((a>b)?a:b)
#define MIN(a,b) ((a<b)?a:b)
#define ABS(a) ((a<0)?(-a):(a))
#define POW2(a) ((a)*(a))
#define DIST(a,b) (sqrtf(POW2(a)+POW2(b)))
#define SIGN(a) ((a<0)?(-1):(1))

#define NULL_COMP_MSG 500

#define ACCEPTABLE_RADIUS_FROM_WAYPOINT 0.1f
#define MAX_TIME_BEFORE_OUT_OF_RANGE 1.5f

#define MAXRSSI 89

static P2PPacket p_reply;
static const uint16_t unlockLow = 100;
static const uint16_t unlockHigh = 300;
//uint8_t currentWayPoint = 0;
//uint8_t currentRecievedWayPoint = 0;
uint16_t STOP = false;
bool HighRSSI = false;
uint16_t START_PROG = false;
bool startedTheProg = false;
uint16_t lostContact = false;
float timeOfLastMsg = 0.0f;

//float initialPos[3] = {0,0,0};
bool HAVE_SENSOR = false;
float temperature_celsius = 0;
int16_t temperature_celsius_comp = 0, humidity_pres_comp = 0;
float humidity_pres = 0;

uint16_t my_up = 2000;

float estimatorX_reset = 0, estimatorY_reset = 0;

float XEstimate = 0;
float YEstimate = 0;

float XOEstimate = 0;
float YOEstimate = 0;

int16_t XEstimate_comp = 0, YEstimate_comp = 0, XOEstimate_comp = 0, YOEstimate_comp = 0;

float velYOther = NULL_COMP_MSG;
float velXOther = NULL_COMP_MSG;

uint16_t took_off = false;

typedef enum {
  ERRORMsg,
  starting,
  blank,
  sayingPos
} HighLevelMsg;
 



void p2pcallbackHandler(P2PPacket *p)
{
  //uint8_t id = p->data[0];
  uint8_t rssi = p->rssi;
  //uint8_t port = p->port;
  uint8_t data0 = p->data[1];
  float timeNow = usecTimestamp() / 1e6;
  DEBUG_PRINT("\nrssi: %d\n", rssi);
  if(lostContact == false){
    if(rssi > MAXRSSI){
      HighRSSI = true;
    }

    if(data0 == (uint8_t)starting){
      DEBUG_PRINT("Other Drone Started Flying\n");
      startedTheProg = true;
    }

    else if(data0 == (uint8_t)sayingPos){
      startedTheProg = true;
      //DEBUG_PRINT("\nI got the pos!\n");
      int16_t x;
      int16_t y;

      int16_t temp, hum;
      uint16_t stop;
      memcpy(&x, &(p->data[2]), sizeof(int16_t));
      memcpy(&y, &(p->data[4]), sizeof(int16_t));
      memcpy(&velXOther, &(p->data[6]), sizeof(float));
      memcpy(&velYOther, &(p->data[10]), sizeof(float));
      memcpy(&stop, &(p->data[14]), sizeof(uint16_t));
      memcpy(&temp, &(p->data[16]), sizeof(int16_t));
      memcpy(&hum, &(p->data[18]), sizeof(int16_t));
      if(!HAVE_SENSOR){
        temperature_celsius_comp = temp;
        temperature_celsius = (float)temp /100;
        humidity_pres_comp = hum;
        humidity = (float)hum /100;
      }
      if(stop){
        STOP = true;
      }
      //DEBUG_PRINT("X: %f, Y:%f, Z:%f\n",(double)x,(double)y,(double)Height);
      //recievedWayPoints[0] = x;
      //recievedWayPoints[1] = y;
      //recievedWayPoints[2] = Height;
      YOEstimate = (float)y / 100;
      XOEstimate = (float)x / 100;
      XOEstimate_comp = x;
      YOEstimate_comp = y;
    }else if (data0 == (uint8_t)blank){
      //DEBUG_PRINT("a blank\n");
    }else{
      //DEBUG_PRINT("non recognized packet!\n");
    }
    timeOfLastMsg = timeNow;
  }
  DEBUG_PRINT("x is: %d",(int)XOEstimate_comp);
}


void sendPacket(HighLevelMsg msg){
    //DEBUG_PRINT("sending packet!, %c",(char)msg);
    p_reply.port=0x00;
    p_reply.size=2*sizeof(uint8_t)+1*sizeof(uint8_t);
    uint64_t address = configblockGetRadioAddress();
    uint8_t my_id =(uint8_t)((address) & 0x00000000ff);
    uint8_t msg_temp = (uint8_t)msg;
    memcpy(&(p_reply.data[0]), &my_id, sizeof(uint8_t));
    memcpy(&(p_reply.data[1]), &msg_temp, sizeof(uint8_t));
    radiolinkSendP2PPacketBroadcast(&p_reply);
}
void sendLocPacket(float x, float y, float height){
    //DEBUG_PRINT("sending packet!, X:%f, Y:%f, Z:%f\n",(double)x, (double)y, (double)height);
    p_reply.port=0x00;
    p_reply.size= 2*sizeof(float)+2*sizeof(uint8_t) + 5*sizeof(uint16_t);
    int16_t x_comp = (int16_t)(x *100);
    int16_t y_comp = (int16_t)(y *100);
    uint64_t address = configblockGetRadioAddress();
    uint8_t my_id =(uint8_t)((address) & 0x00000000ff);
    uint8_t Notempty = (uint8_t)sayingPos;// if the msg is empty or not
    memcpy(&(p_reply.data[0]), &my_id, sizeof(uint8_t));
    memcpy(&(p_reply.data[1]), &Notempty, sizeof(uint8_t));
    memcpy(&(p_reply.data[2]), &x_comp, sizeof(int16_t));
    memcpy(&(p_reply.data[4]), &y_comp, sizeof(int16_t));
    //memcpy(&(p_reply.data[10]), &height, sizeof(float));
    memcpy(&(p_reply.data[6]), &velXOther, sizeof(float));
    memcpy(&(p_reply.data[10]), &velYOther, sizeof(float));
    memcpy(&(p_reply.data[14]), &STOP, sizeof(uint16_t));
    memcpy(&(p_reply.data[16]), &temperature_celsius_comp, sizeof(int16_t));
    memcpy(&(p_reply.data[18]), &humidity_pres_comp, sizeof(int16_t));
    radiolinkSendP2PPacketBroadcast(&p_reply);
}


static State state = idle;

void appMain()
{
  
  p2pRegisterCB(p2pcallbackHandler);

  vTaskDelay(M2T(1000));

  paramVarId_t idHighLevelComm = paramGetVarId("commander", "enHighLevel");
  logVarId_t idUp = logGetVarId("range", "up");
  //logVarId_t idLeft = logGetVarId("range", "left");
  //logVarId_t idRight = logGetVarId("range", "right");
  logVarId_t idFront = logGetVarId("range", "front");
  //logVarId_t idBack = logGetVarId("range", "back");
  logVarId_t idX = logGetVarId("stateEstimate", "x");
  logVarId_t idY = logGetVarId("stateEstimate", "y");
  /*logVarId_t idYaw = logGetVarId("stabilizer", "yaw");*/

  paramVarId_t idPositioningDeck = paramGetVarId("deck", "bcFlow2");
  paramVarId_t idMultiranger = paramGetVarId("deck", "bcMultiranger");
  
  paramSetInt(idHighLevelComm, 1);
  
  uint8_t multirangerInit = paramGetUint(idMultiranger);
  if(!multirangerInit){
    HAVE_SENSOR = true;
    DEBUG_PRINT("I have the sensor!\n");
    Sensorbegin();

  }
  DEBUG_PRINT("starting the project!\n");
  XEstimate = 0;
  XEstimate_comp = 0;
  YEstimate = 0;
  YEstimate_comp = 0;

  //float yaw = 0;

  while(1) {
    vTaskDelay(M2T(200));


    updateVel(velYOther, velXOther, NULL_COMP_MSG);

    uint8_t positioningInit = paramGetUint(idPositioningDeck);
    

    uint16_t my_front = logGetUint(idFront);
    float timeNow = usecTimestamp() / 1e6;
    /*float YawEstimate = logGetFloat(idYaw);*/
    XEstimate = logGetFloat(idX);
    XEstimate_comp = (int16_t)(XEstimate*100);
    YEstimate = logGetFloat(idY);
    YEstimate_comp = (int16_t)(YEstimate*100);

    if (state != moving && state != following && state != end){
      estimatorX_reset = XEstimate;
      estimatorY_reset = YEstimate;
    }

    XEstimate -= estimatorX_reset;
    XEstimate_comp = (int16_t)(XEstimate*100);
    YEstimate -= estimatorY_reset;
    YEstimate_comp = (int16_t)(YEstimate*100);

    float currPos[] = {XEstimate, YEstimate};
    if (STOP){
      state = end;
    }


    if(!positioningInit){
      DEBUG_PRINT("\nFlow deck not connected\n");
      break;
    }
    if(HAVE_SENSOR){
      temperature_celsius = getTemperature();
      humidity_pres = getHumidity();
      temperature_celsius_comp = (int16_t)(temperature_celsius *100);
      humidity_pres_comp = (int16_t)(humidity_pres * 100);
      DEBUG_PRINT("temp: %f, humid: %f\n", (double)temperature_celsius, (double)humidity_pres);

    }else{
      my_up = logGetUint(idUp);
    }

    if (state == moving || state == following || state == end){
      sendLocPacket(XEstimate, YEstimate, HEIGHT);
    }

    //state machine
    if (state == idle){
      if (my_up <= unlockLow || START_PROG){
        DEBUG_PRINT("unlocking...\n");
        state = lowUnlock;

      }
      if((HighRSSI || timeNow-timeOfLastMsg > 2.0f )&& startedTheProg){
        state = unlockedFollower;
        DEBUG_PRINT("High rssi: %d, deltaT is: %f", (int)HighRSSI, (double)(timeNow-timeOfLastMsg));
        lostContact = true;
      }

    }else if(state == lowUnlock){
      if(my_up >= unlockHigh || START_PROG){
        DEBUG_PRINT("starting to fly!\n");
        state = unlocked;
      }

    }else if (state == unlocked){
      MoveMainDrone(state, currPos);
      //vTaskDelay(M2T(500));
      sendPacket(starting);
      DEBUG_PRINT("Hovering!, now moving to first waypoint\n");
      took_off = true;
      DEBUG_PRINT("took off is: %d\n", took_off);
      state = moving;

    }else if (state == unlockedFollower){
      MoveFollowerDrone(state, currPos, my_front);
      DEBUG_PRINT("Hovering!, now moving to first waypoint\n");
      took_off = true;
      state = following;

    }else if(state == moving){
      if (my_up <= unlockLow || STOP){
        DEBUG_PRINT("ending...\n");
        STOP = true;
        state = end;
        continue;
      }
      MoveMainDrone(state, currPos);
      

      /*if (DIST((XEstimate-wayPoints[currentWayPoint][0]),(YEstimate-wayPoints[currentWayPoint][1])) < ACCEPTABLE_RADIUS_FROM_WAYPOINT){
        currentWayPoint++;
        
      }
      if(currentWayPoint >= NUM_OF_WAYPOINTS){ // >= ?
        state = end;
      }*/
      if (STOP){
        state = end;
      }

    }else if(state == end){
      MoveMainDrone(state, currPos);
      break;

    }else if(state == following){
      if (my_up <= unlockLow || STOP){
        DEBUG_PRINT("ending...\n");
        STOP = true;
        state = end;
        continue;
      }
      MoveFollowerDrone(state, currPos, my_front);
    }


  }
  DEBUG_PRINT("ending the program\n");
}


LOG_GROUP_START(my_cf)
LOG_ADD_CORE(LOG_INT16, pos_y, &YEstimate_comp)
LOG_ADD_CORE(LOG_INT16, pos_x, &XEstimate_comp)
LOG_GROUP_STOP(my_cf)

LOG_GROUP_START(other_cf)
LOG_ADD_CORE(LOG_INT16, pos_y, &YOEstimate_comp)
LOG_ADD_CORE(LOG_INT16, pos_x, &XOEstimate_comp)
LOG_ADD_CORE(LOG_INT16, temp, &temperature_celsius_comp)
LOG_ADD_CORE(LOG_INT16, humi, &humidity_pres_comp)
LOG_GROUP_STOP(other_cf)

LOG_GROUP_START(prog_p)
LOG_ADD_CORE(LOG_UINT16, tookOff, &took_off)
LOG_ADD_CORE(LOG_UINT16, lost_contact, &lostContact)
LOG_ADD_CORE(LOG_UINT16, stop, &STOP)
LOG_GROUP_STOP(prog_p)

PARAM_GROUP_START(p)
PARAM_ADD_CORE(PARAM_INT16, start_prog, &START_PROG)
PARAM_ADD_CORE(PARAM_INT16, tookOff, &took_off)
PARAM_ADD_CORE(PARAM_INT16, lost_contact, &lostContact)
PARAM_ADD_CORE(PARAM_INT16, stop, &STOP)
PARAM_ADD_CORE(PARAM_FLOAT, vel_y_other, &velYOther)
PARAM_ADD_CORE(PARAM_FLOAT, vel_x_other, &velXOther)

PARAM_ADD_CORE(PARAM_FLOAT, vel_y_me, &velY_param)
PARAM_ADD_CORE(PARAM_FLOAT, vel_x_me, &velX_param)
PARAM_GROUP_STOP(P)


