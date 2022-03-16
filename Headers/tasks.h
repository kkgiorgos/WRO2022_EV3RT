#pragma once

#include "routes.h"
#include <queue>
#include <string>

//Mission related variables / data

enum items
{
    BOTTLE,
    LAUNDRY_RED,
    LAUNDRY_YELLOW,
    LAUNDRY_BLACK
};

enum tasks
{
    WATER,
    BALL
};

class room
{ 
private:
    enum orientation
    {
        LEFT,
        RIGHT
    };
public:
    enum state
    {
        WAITING,
        SCANNING,
        PICKING_LAUNDRY,
        LEAVING_WATER,
        PLAYING_BALL,
        COMPLETE
    };

private:
    ev3ys::colors color;
    orientation roomOrientation;
    matPos position;
    char name[8];
    tasks task;
    ev3ys::colors laundry;
    state currentState;

public:
    room(ev3ys::colors col);
    void report();

    matPos getPosition();

    void scanTask();
    tasks getTask();

    void pickLaundry();
    ev3ys::colors getLaundryColor();
    
    void leaveWater();

    void pickBall();
    void leaveBall();

    void exitRoom();

    void executeAllActions();
};

extern std::queue<items> rampQueue;
extern ev3ys::colors laundryBaskets[3];
//extern std::map<ev3ys::colors, room> rooms;

//Helper functions

extern matPos startPos;
extern matPos currentPos;
extern orientation currentDirection;

//Actual tasks start here

void startProcedure();

void pickWater();

void scanLaundryBaskets();

void leaveLaundry();

void finishProcedure();
