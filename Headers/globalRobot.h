#pragma once

#include "ev3ys.h"

extern ev3ys::motor ramp;
extern ev3ys::motor grabber;
extern ev3ys::motor leftMotor;
extern ev3ys::motor rightMotor;
extern ev3ys::chassis robot;
extern ev3ys::colorSensor leftSensor;
extern ev3ys::colorSensor rightSensor;
extern ev3ys::colorSensor leftScanner;
extern ev3ys::colorSensor rightScanner;
extern ev3ys::lineFollower lifo, lifoControlled, lifoUnregNormal, lifoUnregExtreme;
extern ev3cxx::Bluetooth bt;
extern ev3ys::timer universalTimer;


extern FILE* bluetooth;

extern bool grabberUsed, startPicking, stopScanning;
extern int roomScanStage;

extern ev3ys::colorSensor *scanner;
extern ev3ys::colors scannedValue;
extern ev3ys::colorSensor *lineDetector;
extern ev3ys::colors roomColor;

#define DEBUG_BLUETOOTH
//#define DEBUG_FILE

#ifdef DEBUG_BLUETOOTH
#define DEBUGPRINT(...)  fprintf(bluetooth, __VA_ARGS__)
#elif defined DEBUG_FILE
#define DEBUGPRINT(...)  fprintf(stderr, __VA_ARGS__)
#else
#define DEBUGPRINT(...)
#endif
