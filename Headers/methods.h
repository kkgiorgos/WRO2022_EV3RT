#pragma once

#include "ev3ys.h"

extern double KP; //2
extern double KI; //2
extern double KD; //200
extern double PIDspeed;

extern double slowKP;
extern double slowKI;
extern double slowKD;

extern double colorCoef;

void resetLifo();
void setLifoNormalReg();
void setLifoSlow();
void setLifoLeft(bool slow = false);
void setLifoLeftExtreme(bool slow = false);
void setLifoRight(bool slow = false);
void setLifoRightExtreme(bool slow = false);

void executeLifoLeftUnlim(int velocity = 50);
void executeLifoRightUnlim(int velocity = 50);

bool detectColorLine(ev3ys::colorSensor &sensor, ev3ys::colors target);
bool detectWhiteRoomBed(ev3ys::colorSensor &sensor);

void align(double time, bool stop = false);
void alignPerpendicular(double time, bool stop = false);
void alignOnMove(double speed);

void reverse(bool stop = false, bool alignEnd = true);
void leftTurn(bool stop = false, bool alignEnd = true);
void rightTurn(bool stop = false, bool alignEnd = true);

void lifo1LineDist(double distance);
void lifo1WhiteLineLeftSlow(double startVelocity, double distance, double slowVelocity = 20, ev3ys::breakMode stopMode = ev3ys::breakMode::BRAKE_COAST);
void lifo1WhiteLineRightSlow(double startVelocity, double distance, double slowVelocity = 20, ev3ys::breakMode stopMode = ev3ys::breakMode::BRAKE_COAST);

void openGrabber();
void openGrabberAsync();
void pickBlock();
void pickBlockStage1();
void pickBlockStage2();

void openGrabberAndPickBlock();

void emptyRampLaundry();
void emptyRampWater();
void emptyRampWaterStage1(bool wait = true);
void emptyRampWaterStage2();

ev3ys::colors scanLaundryBlock(ev3ys::colorSensor &scanner);
ev3ys::colors scanCodeBlock(ev3ys::colorSensor &scanner);
