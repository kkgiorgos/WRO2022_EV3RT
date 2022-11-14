#pragma once

#include "ev3ys.h"

void setLifo(const char *leftPos, const char *rightPos);

void align(double time, bool stop = false);
void alignPerpendicular(double time, bool stop = false);
void alignOnMove(double speed);

void emptyRampLaundry();
void emptyRampWaterStage1(bool wait = true);
void emptyRampWaterStage2();

//Used in general graph traversal for surprise rule not base program
enum lifoRobotPosition
{
    LEFT_OF_LINE,
    CENTERED,
    RIGHT_OF_LINE,
    OTHER
};

enum lineDetectionMode
{
    NO_DETECT,
    COLORED,
    NORMAL,
    SCANNER,
    SPECIAL_REF //DERPECEATED
};

void reverse(lifoRobotPosition startAlignment, lifoRobotPosition endAlignment, ev3ys::breakMode stopMode = ev3ys::breakMode::COAST);
void leftTurn(lifoRobotPosition endAlignment, ev3ys::breakMode stopMode = ev3ys::breakMode::COAST);
void rightTurn(lifoRobotPosition endAlignment, ev3ys::breakMode stopMode = ev3ys::breakMode::COAST);

void switchLifoRobotPosition(double speed, lifoRobotPosition startAlignment, lifoRobotPosition endAlignment);

ev3ys::colors lifoRoute1Line(lifoRobotPosition alignment, double totalDistance, double extremePhase, double slowPhase, double controlledPhase, double maxSpeed, lineDetectionMode detectLine, const char *lifoTarget = "50", ev3ys::breakMode stopMode = ev3ys::breakMode::COAST);
