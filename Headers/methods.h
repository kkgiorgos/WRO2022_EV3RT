#pragma once

#include "ev3ys.h"

extern double KP; //2
extern double KI; //2
extern double KD; //200
extern double PIDspeed;

extern double slowKP;
extern double slowKI;
extern double slowKD;

void setLifo(const char *leftPos, const char *rightPos);

void resetLifo();
void setLifoNormalReg();
void setLifoSlow();
void setLifoLeft(bool slow = false);
void setLifoLeftExtreme(bool slow = false);
void setLifoRight(bool slow = false);
void setLifoRightExtreme(bool slow = false);

void executeLifoLeftUnlim(int velocity = 50);
void executeLifoRightUnlim(int velocity = 50);

void correctionBeforeMovement();
void correctionOnTheMove();

void align(double time, bool stop = false);
void alignPerpendicular(double time, bool stop = false);
void alignOnMove(double speed);

void lifo1WhiteLineLeftSlow(double startVelocity, double distance, double slowVelocity = 20, ev3ys::breakMode stopMode = ev3ys::breakMode::BRAKE_COAST);
void lifo1WhiteLineRightSlow(double startVelocity, double distance, double slowVelocity = 20, ev3ys::breakMode stopMode = ev3ys::breakMode::BRAKE_COAST);

void emptyRampLaundry();
void emptyRampWaterStage1(bool wait = true);
void emptyRampWaterStage2();

//Used in general graph traversal for surprise rule not base program
enum lifoRobotPosition
{
    LEFT_OF_LINE,
    CENTERED,
    RIGHT_OF_LINE
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

ev3ys::colors lifoRoute1Line(lifoRobotPosition alignment, double totalDistance, double extremePhase, double slowPhase, double controlledPhase, double maxSpeed, lineDetectionMode detectLine, int lifoTarget = 50, ev3ys::breakMode stopMode = ev3ys::breakMode::COAST);

void lifo1LineDist(lifoRobotPosition alignment, double totalDistance, double startPhaseDist = 10, double endPhaseDist = 10, double slowDist = 5, lineDetectionMode detectLine = NORMAL, ev3ys::breakMode stopMode = ev3ys::breakMode::COAST);
