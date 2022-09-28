#pragma once

#include "routes.h"
#include <map>
#include <queue>
#include <list>
#include <string>

//Mission related variables / data

enum items
{
    BOTTLE,
    LAUNDRY_RED,
    LAUNDRY_YELLOW,
    LAUNDRY_BLACK
};

enum baskets
{
    BASKET_LEFT,
    BASKET_MIDDLE,
    BASKET_RIGHT
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
        GREEN_YELLOW,
        RED_BLUE
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
    bool doLaundry;
    state currentState;

    void scanLaundry();
    void pickLaundry(int stage);

    void leaveWater(int stage);

    void pickBall(int stage);
    void leaveBall();

    void taskWater();
    void taskWaterLaundry();
    void taskBall();
    void taskBallLaundry();

public:
    room() : currentState(WAITING) {}
    room(ev3ys::colors col);
    void report();

    matPos getPosition();

    void setTask(ev3ys::colors code);
    tasks getTask();

    ev3ys::colors getLaundryColor();

    void enterRoom();

    void executeTask();

    void exitRoom();

    void executeAllActions();
};

extern std::queue<items, std::list<items>> rampQueue;
extern ev3ys::colors laundryBaskets[3];
extern std::map<ev3ys::colors, room> rooms;

//Helper functions
void printRampQueue();
tasks findTask(ev3ys::colors color);
ev3ys::colors findColorOfItem(items item);
items findItem(ev3ys::colors color);
baskets findBasket(ev3ys::colors color);
ev3ys::colors findTheLastColor(ev3ys::colors *cols, int numOfCols = 3);

ev3ys::colors analyzeFrequency(std::map<ev3ys::colors, int> appearances, ev3ys::colors base);

ev3ys::colors scanLaundryBlock(ev3ys::colorSensor &scanner);
ev3ys::colors scanCodeBlock(ev3ys::colorSensor &scanner);
ev3ys::colors scanLaundryBasket(ev3ys::colorSensor &scanner);

bool detectColorLine(ev3ys::colorSensor &sensor, ev3ys::colors target);
bool detectWhiteRoomBed(ev3ys::colorSensor &sensor);

void turnToBasket(baskets current, baskets target);

extern matPos startPos;
extern matPos currentPos;
extern orientation currentDirection;

//Actual tasks start here

void startProcedure();

void pickWater();
void pickWaterTriple();
void pickWaterLast();

void scanLaundryBaskets();

void leaveLaundry();

void finishProcedure();
