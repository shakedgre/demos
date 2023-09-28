#ifndef CONSTS
#define CONSTS

#define NUM_OF_WAYPOINTS 5
#define MAX_NUM_OF_WAY_POINTS 10
#define HEIGHT 0.2f

typedef enum {
    idle,
    lowUnlock,
    lowUnlockFollower,
    unlocked,
    unlockedFollower,
    moving,
    following,
    hover,
    end,
} State;

#endif