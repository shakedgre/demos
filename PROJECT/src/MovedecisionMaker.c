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

#include "stabilizer.h"
#include "estimator_kalman.h"
#include "stabilizer_types.h"




static bool isInit = false;

// Log and param ids
static logVarId_t logIdStateEstimateX;
static logVarId_t logIdStateEstimateY;
static logVarId_t logIdStateEstimateZ;
static logVarId_t logIdKalmanVarPX;
static logVarId_t logIdKalmanVarPY;
static logVarId_t logIdKalmanVarPZ;

static paramVarId_t paramIdStabilizerController;
static paramVarId_t paramIdCommanderEnHighLevel;

static void enableHighlevelCommander() { paramSetInt(paramIdCommanderEnHighLevel, 1); }



#include "controller.h"

#define MAX(a,b) ((a>b)?a:b)
#define MIN(a,b) ((a<b)?a:b)
#define ABS(a) ((a<0)?(-a):(a))
#define POW2(a) ((a)*(a))
#define DIST(a,b) (sqrtf(POW2(a)+POW2(b)))
#define SIGN(a) ((a<0)?(-1):(1))


#define MAX_DIST_FROM_WALL 50
//#define HEIGHT 0.5f


float velX, velY;


void initCommander() {
  if (isInit) {
    return;
  }

  // Get log and param ids
  logIdStateEstimateX = logGetVarId("stateEstimate", "x");
  logIdStateEstimateY = logGetVarId("stateEstimate", "y");
  logIdStateEstimateZ = logGetVarId("stateEstimate", "z");
  logIdKalmanVarPX = logGetVarId("kalman", "varPX");
  logIdKalmanVarPY = logGetVarId("kalman", "varPY");
  logIdKalmanVarPZ = logGetVarId("kalman", "varPZ");

  paramIdStabilizerController = paramGetVarId("stabilizer", "controller");
  paramIdCommanderEnHighLevel = paramGetVarId("commander", "enHighLevel");

  enableHighlevelCommander();

  isInit = true;
}

/*void MoveFollowerDrone(State state, float targetX, float targetY, int16_t frontDist){
    if(state == unlockedFollower){
        crtpCommanderHighLevelTakeoff(HEIGHT, 0.7);
        vTaskDelay(M2T(700));
        //DEBUG_PRINT("Hovering!, now moving to first waypoint\n");
    }
    else if(state == following){
        //calculateVelToGoal(currPos[0], currPos[1], endPos[0], endPos[1], &velX, &velY);
        crtpCommanderHighLevelGoTo(targetX, targetY, HEIGHT, 0.0, 3, false);

    }else if(state == end){
        DEBUG_PRINT("landing\n");
        crtpCommanderHighLevelLand(0.2, 1.0);
        vTaskDelay(M2T(1000));
        crtpCommanderHighLevelStop();
        vTaskDelay(M2T(1000));

    }else{
        DEBUG_PRINT("ERROR: UNRECOGNIZED MOVEMENT COMMAND\n");
        crtpCommanderHighLevelLand(0.2, 1.0);
        vTaskDelay(M2T(1000));
        crtpCommanderHighLevelStop();
        vTaskDelay(M2T(1000));
    }
}*/

void MoveMainDrone(State state, float targetX, float targetY){
    if(state == unlocked){

        crtpCommanderHighLevelTakeoff(HEIGHT, 0.7);
        vTaskDelay(M2T(700));
        //DEBUG_PRINT("Hovering!, now moving to first waypoint\n");
    }else if(state == moving){
        //calculateVelToGoal(currPos[0], currPos[1], checkPoints[currentWayPoint][0], checkPoints[currentWayPoint][1], &velX, &velY);
        crtpCommanderHighLevelGoTo(targetX, targetY, HEIGHT, 0.0, 1.5, false);

    }else if(state == end){
        DEBUG_PRINT("landing\n");
        crtpCommanderHighLevelLand(0.2, 1.0);
        vTaskDelay(M2T(1000));
        crtpCommanderHighLevelStop();
        vTaskDelay(M2T(1000));

    }else{
        DEBUG_PRINT("ERROR: UNRECOGNIZED MOVEMENT COMMAND\n");
        crtpCommanderHighLevelLand(0.2, 1.0);
        crtpCommanderHighLevelStop();
    }
}



/*#include <string.h>
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
//#define HEIGHT 0.5f


static setpoint_t setpoint;

float velX, velY;


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

void updateVel(float velY_param, float velX_param ,float velYother, float velXother, int NULL_MSG){
    velX = velX_param;
    velY = velY_param;
    if (velXother != NULL_MSG){
        velX = velXother;
    }
    if (velYother != NULL_MSG){
        velY = velYother;
    }
}

void MoveFollowerDrone(State state, float currPos[2], int16_t frontDist){
    if(state == unlockedFollower){
        setHoverSetpoint(&setpoint, 0.0f, 0.0f, HEIGHT, 0.0f,false);
        commanderSetSetpoint(&setpoint, 3);
        //DEBUG_PRINT("Hovering!, now moving to first waypoint\n");
    }
    else if(state == following){
        if (frontDist > 5){
            //calculateVelToGoal(currPos[0], currPos[1], endPos[0], endPos[1], &velX, &velY);

            setHoverSetpoint(&setpoint, velX, velY, HEIGHT, 0.0f, false);
            commanderSetSetpoint(&setpoint, 3);

        }else{//if there is a wall
            shortWallFollower();
        }
    }
    else if(state == hover){
        setHoverSetpoint(&setpoint, 0.0f, 0.0f, HEIGHT, 0.0f,false);
        commanderSetSetpoint(&setpoint, 3);
        //DEBUG_PRINT("Hovering!\n");

    }else if(state == end){
        DEBUG_PRINT("landing\n");
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

void MoveMainDrone(State state, float currPos[2]){
    if(state == unlocked){
        setHoverSetpoint(&setpoint, 0.0f, 0.0f, HEIGHT, 0.0f,false);
        commanderSetSetpoint(&setpoint, 3);
        //DEBUG_PRINT("Hovering!, now moving to first waypoint\n");
    }else if(state == moving){
        //calculateVelToGoal(currPos[0], currPos[1], checkPoints[currentWayPoint][0], checkPoints[currentWayPoint][1], &velX, &velY);
        setHoverSetpoint(&setpoint, velX, velY, HEIGHT, 0.0f, false);
        commanderSetSetpoint(&setpoint, 3);

    }else if(state == end){
        DEBUG_PRINT("landing\n");
        memset(&setpoint, 0, sizeof(setpoint_t));
        commanderSetSetpoint(&setpoint, 5);
        vTaskDelay(M2T(1000));

    }else{
        DEBUG_PRINT("ERROR: UNRECOGNIZED MOVEMENT COMMAND\n");
        memset(&setpoint, 0, sizeof(setpoint_t));
        commanderSetSetpoint(&setpoint, 5);
        vTaskDelay(M2T(1000));
    }
}*/


