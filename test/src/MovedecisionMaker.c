#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>

#include "MovedecisionMaker.h"
#include "wall_following.h"
#include "CONSTS.h"

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


#define MAX_DIST_FROM_WALL 50
#define HEIGHT 0.2f


static setpoint_t setpoint;



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

void calculateVelToGoal(float currentX, float currentY, float goalX, float goalY, float* velX, float* velY){
  float maxSpeed = 0.3;
  float dy = goalY - currentY;
  float dx = goalX - currentX;
  
  float dist = DIST(dx,dy);
  (*velX) = maxSpeed*dx/dist;
  (*velY) = maxSpeed*dy/dist;
}

void MoveFollowerDrone(State state, float currPos[2], float endPos[3], int16_t frontDist){
    if(state == unlockedFollower){
        setHoverSetpoint(&setpoint, 0.0f, 0.0f, HEIGHT, 0.0f,false);
        commanderSetSetpoint(&setpoint, 3);
        DEBUG_PRINT("Hovering!, now moving to first waypoint\n");
    }
    if(state == following){
        if (frontDist > MAX_DIST_FROM_WALL){
            float velX, velY;
            calculateVelToGoal(currPos[0], currPos[1], endPos[0], endPos[1], &velX, &velY);

            setHoverSetpoint(&setpoint, velX, velY, HEIGHT, 0.0f, false);
            commanderSetSetpoint(&setpoint, 3);

        }else{//if there is a wall
            shortWallFollower();
        }
    }
    if(state == hover){
        setHoverSetpoint(&setpoint, 0.0f, 0.0f, HEIGHT, 0.0f,false);
        commanderSetSetpoint(&setpoint, 3);
        DEBUG_PRINT("Hovering!\n");

    }else if(state == end){
        DEBUG_PRINT("landing\n");
        setHoverSetpoint(&setpoint, 0.0f, 0.0f, HEIGHT/2, 0.0f, false);
        commanderSetSetpoint(&setpoint, 3);
        vTaskDelay(M2T(200));
        memset(&setpoint, 0, sizeof(setpoint_t));
        commanderSetSetpoint(&setpoint, 5);
        vTaskDelay(M2T(1000));

    }else{
        DEBUG_PRINT("ERROR: UNRECOGNIZED MOVEMENT COMMAND\n");
        memset(&setpoint, 0, sizeof(setpoint_t));
        commanderSetSetpoint(&setpoint, 5);
        vTaskDelay(M2T(1000));
    }
}

void MoveMainDrone(State state, float currPos[2], float checkPoints[MAX_NUM_OF_WAY_POINTS][3], uint8_t currentWayPoint){
    if(state == unlocked){
        setHoverSetpoint(&setpoint, 0.0f, 0.0f, HEIGHT, 0.0f,false);
        commanderSetSetpoint(&setpoint, 3);
        DEBUG_PRINT("Hovering!, now moving to first waypoint\n");
    }
    if(state == moving){
        float velX, velY;
        calculateVelToGoal(currPos[0], currPos[1], checkPoints[currentWayPoint][0], checkPoints[currentWayPoint][1], &velX, &velY);
        setHoverSetpoint(&setpoint, velX, velY, HEIGHT, 0.0f, false);
        commanderSetSetpoint(&setpoint, 3);

    }else if(state == end){
        DEBUG_PRINT("landing\n");
        setHoverSetpoint(&setpoint, 0.0f, 0.0f, HEIGHT/2, 0.0f, false);
        commanderSetSetpoint(&setpoint, 3);
        vTaskDelay(M2T(200));
        memset(&setpoint, 0, sizeof(setpoint_t));
        commanderSetSetpoint(&setpoint, 5);
        vTaskDelay(M2T(1000));

    }else{
        DEBUG_PRINT("ERROR: UNRECOGNIZED MOVEMENT COMMAND\n");
        memset(&setpoint, 0, sizeof(setpoint_t));
        commanderSetSetpoint(&setpoint, 5);
        vTaskDelay(M2T(1000));
    }
}