#include "CONSTS.h"
#ifndef MOVE_DECISION_MAKER
#define MOVE_DECISION_MAKER

typedef signed short int16_t;

void MoveFollowerDrone(State state, float currPos[2], float endPos[3], int16_t frontDist);

void MoveMainDrone(State state, float currPos[2], float checkPoints[MAX_NUM_OF_WAY_POINTS][3], uint8_t currentWayPoint);

#endif