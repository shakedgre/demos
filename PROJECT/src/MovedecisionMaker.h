#include "CONSTS.h"
#ifndef MOVE_DECISION_MAKER
#define MOVE_DECISION_MAKER

typedef signed short int16_t;

float velX_param, velY_param;

void updateVel(float velYother, float velXother, int NULL_MSG);

void MoveFollowerDrone(State state, float currPos[2], int16_t frontDist);

void MoveMainDrone(State state, float currPos[2]);

#endif