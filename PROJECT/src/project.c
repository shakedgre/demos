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

#define HEIGHT 0.2f
#define ACCEPTABLE_RADIUS_FROM_WAYPOINT 0.1f
#define NUM_OF_WAYPOINTS 3

static const uint16_t unlockLow = 100;
static const uint16_t unlockHigh = 300;
uint8_t currentWayPoint = 0;
//TO DO!!: add an if which checkes id and change initial position accordingly
float initialPos[3] = {0,0,0};

float wayPoints[NUM_OF_WAYPOINTS][3] = {{0.4f,0.4f,HEIGHT},
                                        {0.4f,0.0f,HEIGHT},
                                        {0,0,HEIGHT}};//global [x,y,z]

void p2pcallbackHandler(P2PPacket *p)
{
  uint8_t id = p->data[0];
  uint8_t rssi = p->rssi;
  uint8_t port = p->port;
  uint8_t data0 = p->data[0];

  DEBUG_PRINT("\n\nthe id: %x\nthe rssi: %d\nthe port: %d\n data[0]: %x",id,rssi,port,data0);

}

void calculateVelToGoal(float currentX, float currentY, float goalX, float goalY, float* velX, float* velY){
  float maxSpeed = 0.3;
  float dy = goalY - currentY;
  float dx = goalX - currentX;
  
  float dist = DIST(dx,dy);
  (*velX) = maxSpeed*dx/dist;
  (*velY) = maxSpeed*dy/dist;
}

float calculateYaw(float goalX, float goalY, float currX, float currY){
  float dy = goalY - currY;
  float dx = goalX - currX;
  float tanTheta = ABS(dy/dx);
  float theta = atanf(tanTheta) * 180.0f/3.14159f;
  return (SIGN(dy)*(theta + 90*(SIGN(dx)-1)))>0?(SIGN(dy)*(theta + 90*(SIGN(dx)-1))):(360-(SIGN(dy)*(theta + 90*(SIGN(dx)-1))));

}

static void setHoverSetpoint(setpoint_t *setpoint, float vx, float vy, float z, float yawrate, bool relative)
{
  setpoint->mode.z = modeAbs;
  setpoint->position.z = z;


  setpoint->mode.yaw = modeAbs;
  setpoint->attitudeRate.yaw = yawrate;


  setpoint->mode.x = modeVelocity;
  setpoint->mode.y = modeVelocity;
  setpoint->velocity.x = vx;
  setpoint->velocity.y = vy;

  setpoint->velocity_body = relative;
}



typedef enum {
    idle,
    lowUnlock,
    unlocked,
    moving,
    end
} State;

static State state = idle;



void appMain()
{
  static setpoint_t setpoint;

  p2pRegisterCB(p2pcallbackHandler);

  vTaskDelay(M2T(3000));
  

  paramVarId_t idHighLevelComm = paramGetVarId("commander", "enHighLevel");
  logVarId_t idUp = logGetVarId("range", "up");
  /*logVarId_t idLeft = logGetVarId("range", "left");
  logVarId_t idRight = logGetVarId("range", "right");
  logVarId_t idFront = logGetVarId("range", "front");
  logVarId_t idBack = logGetVarId("range", "back");*/
  logVarId_t idX = logGetVarId("stateEstimate", "x");
  logVarId_t idY = logGetVarId("stateEstimate", "y");
  /*logVarId_t idYaw = logGetVarId("stabilizer", "yaw");*/

  paramVarId_t idPositioningDeck = paramGetVarId("deck", "bcFlow2");
  paramVarId_t idMultiranger = paramGetVarId("deck", "bcMultiranger");
  
  paramSetInt(idHighLevelComm, 1);
  

  DEBUG_PRINT("starting the program!\n");
  float XEstimate = initialPos[0];
  float YEstimate = initialPos[1];

  float remTime = 0;
  float yaw = 0;
  while(1) {
    vTaskDelay(M2T(50));
    //DEBUG_PRINT(".");

    uint8_t positioningInit = paramGetUint(idPositioningDeck);
    uint8_t multirangerInit = paramGetUint(idMultiranger);
    uint16_t my_up = logGetUint(idUp);

    /*float YawEstimate = logGetFloat(idYaw);*/


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
      DEBUG_PRINT("currently idle...");
      if (my_up <= unlockLow){
        DEBUG_PRINT("unlocking...\n");
        state = lowUnlock;
      }
    }else if(state == lowUnlock){
      if(my_up >= unlockHigh){
        DEBUG_PRINT("flying!\n");
        state = unlocked;
      }
    }else if (state == unlocked){
      setHoverSetpoint(&setpoint, 0.0f, 0.0f, HEIGHT, 20.0f,false);
      commanderSetSetpoint(&setpoint, 3);
      vTaskDelay(M2T(500));
      DEBUG_PRINT("Hovering!, now moving to first waypoint\n");
      state = moving;
    }else if(state == moving){
      if (my_up <= unlockLow){
        DEBUG_PRINT("ending...\n");
        state = end;
        continue;
      }
      XEstimate = logGetFloat(idX) + initialPos[0];
      YEstimate = logGetFloat(idY) + initialPos[1];
      float Xvel, Yvel;
      calculateVelToGoal(XEstimate,YEstimate,wayPoints[currentWayPoint][0],wayPoints[currentWayPoint][1],&Xvel,&Yvel);
      yaw = calculateYaw(wayPoints[currentWayPoint][0], wayPoints[currentWayPoint][1], XEstimate, YEstimate);
      float timeNow = usecTimestamp() / 1e6;
      if (remTime == 0.0f){
        remTime = timeNow;
      }
      

      setHoverSetpoint(&setpoint, Xvel, Yvel, HEIGHT, yaw, false);
      commanderSetSetpoint(&setpoint, 3);
      if (DIST((XEstimate-wayPoints[currentWayPoint][0]),(YEstimate-wayPoints[currentWayPoint][1])) > ACCEPTABLE_RADIUS_FROM_WAYPOINT){ continue;}
      remTime = timeNow;
      //if(DIST((XEstimate-wayPoints[currentWayPoint][0]),(YEstimate-wayPoints[currentWayPoint][1])) > ACCEPTABLE_RADIUS_FROM_WAYPOINT) {continue;}
      currentWayPoint++;

      if(currentWayPoint >= NUM_OF_WAYPOINTS){
        state = end;
      }

    }else if(state == end){
      DEBUG_PRINT("landing\n");
      setHoverSetpoint(&setpoint, 0.0f, 0.0f, HEIGHT/2, 0.0f, false);
      commanderSetSetpoint(&setpoint, 3);
      vTaskDelay(M2T(200));
      memset(&setpoint, 0, sizeof(setpoint_t));
      commanderSetSetpoint(&setpoint, 3);
      break;
    }
    

  }
  DEBUG_PRINT("ending the program\n");
}
