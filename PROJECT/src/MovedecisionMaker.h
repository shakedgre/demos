#include "CONSTS.h"
#ifndef MOVE_DECISION_MAKER
#define MOVE_DECISION_MAKER

typedef signed short int16_t;


void initCommander();

void MoveMainDrone(State state, float targetX, float targetY);

#endif