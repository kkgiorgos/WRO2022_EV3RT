#pragma once

#include "ev3ys.h"

extern double KP; //2
extern double KI; //2
extern double KD; //200
extern double PIDspeed;

extern double colorCoef;

void resetLifo();
void setLifoLeft();
void setLifoLeftExtreme();
void setLifoRight();
void setLifoRightExtreme();

void executeLifoLeftUnlim(int velocity = 50);
void executeLifoRightUnlim(int velocity = 50);

bool detectColorLine(ev3ys::colorSensor &sensor, ev3ys::colors target);

void align(double time, bool stop = false);
void alignPerpendicular(double time, bool stop = false);
void alignOnMove(double speed);

void reverse(bool stop = false, bool alignEnd = true);
void leftTurn(bool stop = false, bool alignEnd = true);
void rightTurn(bool stop = false, bool alignEnd = true);

void lifo1LineDist(double distance);

void openGrabber();
void pickBlock();

void emptyRampLaundry();
void emptyRampWater();

ev3ys::colors scanLaundryBlock(ev3ys::colorSensor &scanner);
ev3ys::colors scanCodeBlock(ev3ys::colorSensor &scanner);
