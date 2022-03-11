#pragma once

#include "ev3ys.h"

extern double KP; //2
extern double KI; //2
extern double KD; //200
extern double colorCoef;

void resetLifo();

void align(double time, bool stop = false);
void alignOnMove(double speed);

void reverse(bool stop = false, bool alignEnd = true);
void leftTurn(bool stop = false, bool alignEnd = true);
void rightTurn(bool stop = false, bool alignEnd = true);

void lifo1LineDist(double distance);
