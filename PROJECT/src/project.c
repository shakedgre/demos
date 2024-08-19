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
bool startedTheProg = false;
uint16_t lostContact = false;
float timeOfLastMsg = 0.0f;

//float initialPos[3] = {0,0,0};

uint16_t my_up = 2000;

float XEstimate = 0;
float YEstimate = 0;

float XOEstimate = 0;
float YOEstimate = 0;

float velYOther = NULL_COMP_MSG;
float velXOther = NULL_COMP_MSG;

uint16_t took_off = false;

/*void setInitPos(float* initialPos){
    uint64_t address = configblockGetRadioAddress();
    uint8_t my_id =(uint8_t)((address) & 0x00000000ff);
    if(my_id == 0xE7){
      initialPos[0] = 0.0f;
      initialPos[1] = 0.0f;
      initialPos[2] = 0.0f;
    }else if(my_id == 0xE6){
      initialPos[0] = 0.0f;
      initialPos[1] = 0.2f;
      initialPos[2] = 0.0f;
    }
}


float wayPoints[MAX_NUM_OF_WAY_POINTS][3] = {{0,0,HEIGHT},
                                        {2.0f,0.0f,HEIGHT},
                                        {2.0f,-1.0f,HEIGHT},
                                        {2.0f,-2.0f,HEIGHT},
                                        {2.0f,-2.5f,HEIGHT}};//global [x,y,z]
*/
//float recievedWayPoints[3];

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
      float x;
      float y;
      float Height;
      uint16_t stop;
      memcpy(&x, &(p->data[2]), sizeof(float));
      memcpy(&y, &(p->data[6]), sizeof(float));
      memcpy(&Height, &(p->data[10]), sizeof(float));
      memcpy(&velXOther, &(p->data[14]), sizeof(float));
      memcpy(&velYOther, &(p->data[18]), sizeof(float));
      memcpy(&stop, &(p->data[22]), sizeof(uint16_t));
      if(stop){
        STOP = true;
      }
      //DEBUG_PRINT("X: %f, Y:%f, Z:%f\n",(double)x,(double)y,(double)Height);
      //recievedWayPoints[0] = x;
      //recievedWayPoints[1] = y;
      //recievedWayPoints[2] = Height;
      YOEstimate = y;
      XOEstimate = x;
    }else if (data0 == (uint8_t)blank){
      //DEBUG_PRINT("a blank\n");
    }else{
      //DEBUG_PRINT("non recognized packet!\n");
    }
    timeOfLastMsg = timeNow;
  }

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
    p_reply.size= 5*sizeof(float)+2*sizeof(uint8_t) + sizeof(uint16_t);
    uint64_t address = configblockGetRadioAddress();
    uint8_t my_id =(uint8_t)((address) & 0x00000000ff);
    uint8_t Notempty = (uint8_t)sayingPos;// if the msg is empty or not
    memcpy(&(p_reply.data[0]), &my_id, sizeof(uint8_t));
    memcpy(&(p_reply.data[1]), &Notempty, sizeof(uint8_t));
    memcpy(&(p_reply.data[2]), &x, sizeof(float));
    memcpy(&(p_reply.data[6]), &y, sizeof(float));
    memcpy(&(p_reply.data[10]), &height, sizeof(float));
    memcpy(&(p_reply.data[14]), &velXOther, sizeof(float));
    memcpy(&(p_reply.data[18]), &velYOther, sizeof(float));
    memcpy(&(p_reply.data[22]), &STOP, sizeof(uint16_t));
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
  

  DEBUG_PRINT("starting the project!\n");
  XEstimate = 0;
  YEstimate = 0;

  //float yaw = 0;

  while(1) {
    vTaskDelay(M2T(200));

    updateVel(velYOther, velXOther, NULL_COMP_MSG);

    uint8_t positioningInit = paramGetUint(idPositioningDeck);
    uint8_t multirangerInit = paramGetUint(idMultiranger);
    my_up = logGetUint(idUp);
    uint16_t my_front = logGetUint(idFront);
    float timeNow = usecTimestamp() / 1e6;
    /*float YawEstimate = logGetFloat(idYaw);*/
    XEstimate = logGetFloat(idX);
    YEstimate = logGetFloat(idY);
    float currPos[] = {XEstimate, YEstimate};
    if (STOP){
      state = end;
    }


    if(!positioningInit){
      DEBUG_PRINT("\nFlow deck not connected\n");
      break;
    }
    if(!multirangerInit){
      DEBUG_PRINT("\nmultiranger deck not connected\n");
      break;
    }
    //state machine
    if (state == idle){
      if (my_up <= unlockLow){
        DEBUG_PRINT("unlocking...\n");
        state = lowUnlock;

      }
      if((HighRSSI || timeNow-timeOfLastMsg > 2.0f )&& startedTheProg){
        state = unlockedFollower;
        lostContact = true;
      }

    }else if(state == lowUnlock){
      if(my_up >= unlockHigh){
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
      sendLocPacket(XEstimate, YEstimate, HEIGHT);

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
      sendLocPacket(XEstimate,YEstimate,0.0f);
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
      /*if (DIST((XEstimate-recievedWayPoints[0]),(YEstimate-recievedWayPoints[1])) > ACCEPTABLE_RADIUS_FROM_WAYPOINT){ continue;}
      state = hover;
      

    }else if(state == hover){
      DEBUG_PRINT("hovering\n");
      if (my_up <= unlockLow){
        STOP = true;
        state = end;
      }
      MoveFollowerDrone(state, currPos, my_front);
    }*/

  }
  DEBUG_PRINT("ending the program\n");
}



LOG_GROUP_START(my_cf)
LOG_ADD_CORE(LOG_UINT16, up, &my_up)
LOG_ADD_CORE(LOG_FLOAT, pos_y, &YEstimate)
LOG_ADD_CORE(LOG_FLOAT, pos_x, &XEstimate)
LOG_GROUP_STOP(my_cf)

LOG_GROUP_START(other_cf)
LOG_ADD_CORE(LOG_FLOAT, pos_y, &YOEstimate)
LOG_ADD_CORE(LOG_FLOAT, pos_x, &XOEstimate)
LOG_GROUP_STOP(other_cf)

LOG_GROUP_START(prog_p)
LOG_ADD_CORE(LOG_UINT16, tookOff, &took_off)
LOG_ADD_CORE(LOG_UINT16, lost_contact, &lostContact)
LOG_ADD_CORE(LOG_UINT16, stop, &STOP)
LOG_GROUP_STOP(prog_p)

PARAM_GROUP_START(p)
PARAM_ADD_CORE(PARAM_FLOAT, vel_y_other, &velYOther)
PARAM_ADD_CORE(PARAM_FLOAT, vel_x_other, &velXOther)

PARAM_ADD_CORE(PARAM_FLOAT, vel_y_me, &velY_param)
PARAM_ADD_CORE(PARAM_FLOAT, vel_x_me, &velX_param)
PARAM_GROUP_STOP(P)


