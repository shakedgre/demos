#ifndef CONSTS
#define CONSTS

#define HEIGHT 0.5f

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