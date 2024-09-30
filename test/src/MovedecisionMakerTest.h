#include "CONSTS.h"
#ifndef MOVE_DECISION_MAKER
#define MOVE_DECISION_MAKER

typedef signed short int16_t;

float targetX, targetY;

//void updateVel(float velY_param, float velX_param ,float velYother, float velXother, int NULL_MSG);

void initCommander();

void MoveFollowerDrone(State state, float currPos[2], int16_t frontDist);

void MoveMainDrone(State state, float currPos[2]);

#endif