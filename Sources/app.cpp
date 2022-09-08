#include <cstdlib>

#include <vector>
#include <queue>
#include <list>
#include <map>
#include <cstdio>
#include <climits>

#include "routes.h"
#include "tasks.h"
#include "methods.h"

#include "ev3ys.h"
#include "globalRobot.h"
#include "app.h"

using namespace ev3ys;
using namespace ev3cxx;
using namespace std;

//#define CALIBRATION 

Bluetooth bt;
FILE *bluetooth;

motor grabber(MotorPort::A, true);
motor ramp(MotorPort::D, false);
motor leftMotor(MotorPort::B, true, MotorType::LARGE);
motor rightMotor(MotorPort::C, false, MotorType::LARGE);
chassis robot(&leftMotor, &rightMotor, 6.24, 18, 0.1, 0.01);
colorSensor leftSensor(SensorPort::S2, false, "WRO2022");
colorSensor rightSensor(SensorPort::S3, false, "WRO2022");
colorSensor leftScanner(SensorPort::S1, false, "WRO2022");
colorSensor rightScanner(SensorPort::S4, false, "WRO2022");
lineFollower lifo(700, &robot, &leftSensor, &rightSensor);


queue<items, list<items>> rampQueue;
colors laundryBaskets[3];
map<colors, room> rooms;

double KP = 6;      //OLD: 0.5
double KI = 0.6;    //OLD: 0.05
double KD = 60;     //OLD: 5
double PIDspeed = 75;

double slowKP = 2.5;    //3
double slowKI = 1.5;    //1
double slowKD = 80;     //60

double colorCoef = 1;

matPos startPos;
matPos currentPos;
orientation currentDirection;

std::vector<int> graph[V];

BrickButton btnEnter(BrickButtons::ENTER);

void startData()
{
    rooms.insert(pair<colors, room>(RED, room(RED)));
    rooms.insert(pair<colors, room>(GREEN, room(GREEN)));
    rooms.insert(pair<colors, room>(BLUE, room(BLUE)));
    rooms.insert(pair<colors, room>(YELLOW, room(YELLOW)));

    currentPos = S;
}

void init()
{
    freopen("logOut.txt","w+", stderr);
    #ifdef DEBUG_BLUETOOTH
        bluetooth = ev3_serial_open_file(EV3_SERIAL_BT);
    #endif

    robot.setMode(speedMode::CONTROLLED);
    robot.setLinearAccelParams(100, 0, 0);
    robot.setAngularAccelParams(1000, 0, 0);
    robot.setStallTolerance(10, 200, 6, 40, 0.5);

    lifo.setDoubleFollowMode("SL", "SR");
    lifo.initializeMotionMode(speedMode::CONTROLLED);
    lifo.setSensorMode(sensorModes::REFLECTED);
    lifo.setAccelParams(100, 5, 50);
    lifo.setPIDparams(0.5, 0.05, 5, 75);  //UNREGULATED VALUES
    //lifo.setPIDparams(1, 1, 100, 1000);      //REGULATED VALUES

    leftSensor.setNormalisation(true);
    leftSensor.setFiltering(false);
    leftSensor.setCutoffValue(27);
    rightSensor.setNormalisation(true);
    rightSensor.setFiltering(false);
    rightSensor.setCutoffValue(27);

    leftScanner.setFiltering(false);
    rightScanner.setFiltering(false);
    leftScanner.setNormalisation(true);
    rightScanner.setNormalisation(true);
    
    grabber.setMode(REGULATED);
    ramp.setMode(REGULATED);
    //Initialize position of mechanisms (here or after start of mission, e.g. btn press)

    display.format("WAIT FOR SENSORS\n");
    btnEnter.waitForPress();
    act_tsk(INIT_GRAB_TASK);
    act_tsk(INIT_RAMP_TASK);
}

void init_grab_task(intptr_t unused)
{
    //INITIALIZATION OF GRABBER
    //grabber.moveUntilStalled(500, BRAKE, 0.4);
    //timer t;
    //t.secDelay(0.05);
    grabber.moveUntilStalled(-300);
}

void init_ramp_task(intptr_t unused)
{
    //INITIALIZATION OF RAMP
    ramp.moveUntilStalled(800, COAST);
    ramp.moveUntilStalled(-500, BRAKE); 
}

void close_ramp_task(intptr_t unused)
{
    //CLOSES RAMP
    ramp.moveUntilStalled(-400, BRAKE);
}

void open_grabber_task(intptr_t unused)
{
    //OPENS GRABBER
    openGrabber();
}

void main_task(intptr_t unused) 
{
    bt.open();
    format(bt, "\n\rSTARTING\n\r");
    format(bt, "Battery Voltage: %  \n\r")%ev3_battery_voltage_mV();
    init();
    timer missionTimer;

    timer t;

    vector<int> path;
    graphInit();
    startData();

    //Mission Code
    startProcedure();
    
    fullRouteStandard(W);
    pickWater();

    fullRouteStandard(GR);
    rooms[GREEN].executeAllActions();

    // lifo1WhiteLineRightSlow(30, 12, 30, NONE);
    // robot.setLinearAccelParams(200, 30, 30);
    // robot.straight(30, 8, NONE);
    // lifo.initializeMotionMode(CONTROLLED);
    // lifo.setDoubleFollowMode("66", "SR");
    // lifo.setAlignMode(true);
    // lifo.setPIDparams(slowKP*3, slowKI*1.8, slowKD*5, 1);
    // lifo.setAccelParams(200, 30, 20);
    // lifo.distance(20, 7, NONE);
    // lifo.setPIDparams(slowKP*2, slowKI*2, slowKD*2, 1);
    // lifo.distance(20, 8, NONE);

    // robot.setMode(CONTROLLED);
    // while(true)
    // {
    //     robot.setLinearAccelParams(100, 10, 10);
    //     robot.straight(45, 20, NONE);
    //     robot.setLinearAccelParams(100, 10, 20);
    //     robot.arc(20, 15, -9, NONE);
    //     robot.setLinearAccelParams(100, 20, 20);
    //     robot.arc(45, 60, -3, NONE);
    //     robot.setLinearAccelParams(100, 20, 10);
    //     robot.arc(20, 15, -9, NONE);
    // }

    // while(true)
    // {
    //     robot.setLinearAccelParams(600, -100, 100);
    //     robot.turn(300, 90);
    // }   
    

    // robot.setMode(CONTROLLED);
    // robot.setLinearAccelParams(100, 10, 25);
    // robot.straight(45, 40, NONE);
    // robot.setLinearAccelParams(100, 25, 10);
    // robot.arc(45, 90, -9, NONE);
    // robot.setLinearAccelParams(100, 10, 10);
    // robot.straight(45, 10);

    
    // robot.setMode(CONTROLLED);
    // robot.setAngularAccelParams(600, 100, 100);
    // robot.turn(300, 360);
    // robot.setAngularAccelParams(600, -100, 100);
    // robot.turn(300, -360);
    // robot.setAngularAccelParams(600, 100, 100);
    // robot.turn(300, 360);

    // fullRouteStandard(RR);
    // rooms[RED].executeAllActions();
    // fullRouteStandard(BR);
    // rooms[BLUE].executeAllActions();
    // fullRouteStandard(YR);
    // rooms[YELLOW].executeAllActions();
    
    // fullRouteStandard(L);
    // scanLaundryBaskets();
    // leaveLaundry();
    
    // fullRouteStandard(S);
    // finishProcedure();

    // timer::secDelay(0.2);

    // robot.setMode(UNREGULATED);
    // robot.tankUnlim(50, 50, true);
    // display.resetScreen();
    // while(!btnEnter.isPressed())
    // {
    //     robot.tankUnlim(50, 50);
    //     display.format("%  \n\n\n") %ev3_battery_voltage_mV();
    //     timer::secDelay(0.1);
    // }

    // currentPos = CR;
    // currentDirection = EAST;
    // fullRouteStandard(GR);
    // rooms[GREEN].setTask(GREEN);
    // rooms[GREEN].executeAllActions();

    // ev3_motor_config(EV3_PORT_B, LARGE_MOTOR);
    // ev3_motor_config(EV3_PORT_C, LARGE_MOTOR);

    // int setPower = 0;
    // int maxPower = 100;
    // bool isPowerAsc = true;
    // while (true)
    // {
    //     if(isPowerAsc && setPower < maxPower)
    //         setPower++;
    //     else if(!isPowerAsc && setPower > -maxPower)
    //         setPower--;
    //     else if(setPower == maxPower)
    //     {
    //         setPower--;
    //         isPowerAsc = false;
    //     }
    //     else
    //     {
    //         setPower++;
    //         isPowerAsc = true;
    //     }
        
    //     ev3_motor_set_power(EV3_PORT_B, setPower);
    //     ev3_motor_set_power(EV3_PORT_C, -setPower);

    //     tslp_tsk(2);
    // }    
    
    // ev3_motor_config(EV3_PORT_B, LARGE_MOTOR);
    // ev3_motor_config(EV3_PORT_C, LARGE_MOTOR);

    // // FILE *log = fopen("WRO2022/batteryTest.txt", "w");
    // FILE *log = bluetooth;

    // int setPower = 0;
    // int maxPower = 100;
    // bool isPowerAsc = true;
    // int leftReportedPower, rightReportedPower;
    // double leftActualSpeed, rightActualSpeed;
    // int voltage = 8000, current, power;
    // while(voltage > 7500)
    // {
    //     ev3_motor_set_power(EV3_PORT_B, setPower);
    //     ev3_motor_set_power(EV3_PORT_C, setPower);
    //     t.secDelay(0.05);
    //     ev3_motor_reset_counts(EV3_PORT_B);
    //     ev3_motor_reset_counts(EV3_PORT_C);
    //     t.reset();
    //     t.secDelay(0.15);
    //     leftReportedPower = ev3_motor_get_power(EV3_PORT_B);
    //     rightReportedPower = ev3_motor_get_power(EV3_PORT_C);
    //     leftActualSpeed = ev3_motor_get_counts(EV3_PORT_B) / t.secElapsed();
    //     rightActualSpeed = ev3_motor_get_counts(EV3_PORT_C) / t.secElapsed();
    //     voltage = ev3_battery_voltage_mV();
    //     current = ev3_battery_current_mA();
    //     power = voltage * current;


    //     if(setPower == 0)
    //     {
    //         ev3_motor_stop(EV3_PORT_B, false);
    //         ev3_motor_stop(EV3_PORT_C, false);

    //         t.secDelay(5);

    //         voltage = ev3_battery_voltage_mV();
    //         current = ev3_battery_current_mA();
    //         power = voltage * current;
    //         fprintf(log, "%d\t%d\t%d\t%lf\t%lf\t%d\t%d\t%d\n", 0, 0, 0, 0.0, 0.0, voltage, current, power);
    //     }
    //     else
    //     {
    //         fprintf(log, "%d\t%d\t%d\t%lf\t%lf\t%d\t%d\t%d\n", setPower, leftReportedPower, rightReportedPower, leftActualSpeed, rightActualSpeed, voltage, current, power);
    //     }
        
    //     if(isPowerAsc && setPower < maxPower)
    //         setPower++;
    //     else if(!isPowerAsc && setPower > -maxPower)
    //         setPower--;
    //     else if(setPower == maxPower)
    //     {
    //         setPower--;
    //         isPowerAsc = false;
    //     }
    //     else
    //     {
    //         setPower++;
    //         isPowerAsc = true;
    //     }
    // }

    // FILE *log = bluetooth;

    // BrickButton btnUp(BrickButtons::UP);
    // BrickButton btnDown(BrickButtons::DOWN);
    // BrickButton btnLeft(BrickButtons::LEFT);
    // BrickButton btnRight(BrickButtons::RIGHT);

    // robot.setMode(REGULATED);
    // robot.setUnregulatedDPS(true);
    // lifo.initializeMotionMode(UNREGULATED);
    // lifo.setDoubleFollowMode("SL", "SR");
    // lifo.setSensorMode(sensorModes::REFLECTED);
    // lifo.setAlignMode(true);

    // int velocity = 600;//550
    // double maxSpeed = 45;
    // double k_values[3] = {8, 0, 0};
    // int currentValue = 0;
    // double increments[3] = {0.1, 0.01, 1};
    // double velocityKp = 5;
    // double velocityError;
    // double measuredVelocity;
    // double givenVelocity;
    // int position = 0;
    // while(true)
    // {
    //     currentValue = 0;
    //     while(!btnEnter.isPressed())
    //     {
    //         if(btnLeft.isPressed())
    //             k_values[currentValue] -= increments[currentValue];
    //         else if(btnRight.isPressed())
    //             k_values[currentValue] += increments[currentValue];
    //         else if(btnUp.isPressed())
    //             currentValue = (currentValue + 2) % 3;
    //         else if(btnDown.isPressed())
    //             currentValue = (currentValue + 1) % 3;

    //         tslp_tsk(100);
    //         display.resetScreen(); 
    //         display.format("Kp: %  \nKi: %  \nKd: %  ") %k_values[0] %k_values[1] %k_values[2];
    //     }
    //     lifo.setPIDparams(k_values[0], k_values[1], k_values[3], 900);
    //     t.secDelay(0.5);
    //     lifo.unlimited(velocity, true); 
    //     t.reset();
    //     // position = 0;
    //     // while(position + robot.getPosition() < 70)
    //     // {
    //     //     measuredVelocity = robot.getPosition() / t.secElapsed();
    //     //     velocityError = velocity - measuredVelocity;
    //     //     givenVelocity = velocity + velocityError * velocityKp;
    //     //     lifo.unlimited(givenVelocity);
    //     //     fprintf(log, "%lf\n", robot.getPosition()/t.secElapsed());
    //     //     if(t.secElapsed() > 0.2)
    //     //     {
    //     //         t.reset();
    //     //         position += robot.getPosition();
    //     //         robot.resetPosition();
    //     //     }
    //     // }

    //     lifo.setDoubleFollowMode("SR", "50");
    //     lifo.initializeMotionMode(UNREGULATED);
    //     lifo.setAlignMode(true);
    //     robot.setUnregulatedDPS(true);
    //     lifo.setPIDparams(6, 0.6, 60, 1);
    //     robot.resetPosition();
    //     int startRotSpeed = 400;
    //     int endRotSpeed = 600;
    //     int speedDiff = endRotSpeed - startRotSpeed;
    //     double dist = 7;
    //     double pos;
    //     while(pos = robot.getPosition() < dist)
    //     {
    //         lifo.unlimited((pos/dist)*speedDiff + startRotSpeed);
    //     }
    //     t.reset();
    //     robot.resetPosition();
    //     lifo.distance(endRotSpeed, 3, NONE);
    //     double currentVelocity = robot.getPosition() / t.secElapsed();
    //     lifo.initializeMotionMode(CONTROLLED);
    //     lifo.setAccelParams(150, currentVelocity, 20);
    //     lifo.setAlignMode(true);
    //     lifo.distance(50, 58, NONE);
    //     lifo.setAccelParams(200, 20, 20);
    //     lifo.lines(20, 1, BRAKE_COAST);
    // }


    // lifo.setDoubleFollowMode("SL", "SR");
    // lifo.initializeMotionMode(UNREGULATED);
    // lifo.setAlignMode(true);
    // robot.setUnregulatedDPS(true);
    // lifo.setPIDparams(6, 0.6, 60, 1);
    // lifo.distance(300, 2, NONE);
    // lifo.distance(400, 2, NONE);
    // lifo.distance(500, 2, NONE);
    // lifo.distance(600, 2, NONE);
    // lifo.distance(700, 2, NONE);
    // lifo.distance(800, 2, NONE);
    // robot.resetPosition();
    // t.reset();
    // lifo.distance(850, 40, NONE);
    // double currentVelocity = robot.getPosition() / t.secElapsed();
    // setLifoSlow();
    // lifo.setAccelParams(250, currentVelocity, 20);
    // lifo.setAlignMode(true);
    // lifo.distance(currentVelocity, 10, NONE);
    // lifo.setAccelParams(200, 20, 20);
    // lifo.distance(20, 3, NONE);
    // lifo.lines(20, 1, BRAKE_COAST);

    // lifo1WhiteLineLeftSlow(30, 10, 20, NONE);
    // robot.setLinearAccelParams(200, 20, 20);
    // robot.straight(20, 8, NONE);
    // lifo1WhiteLineLeftSlow(20, 10);

    robot.stop(BRAKE);

    /*robot.stop(BRAKE);
    btnEnter.waitForPress();
    rampQueue.pop();
    rampQueue.pop();

    rampQueue.push(LAUNDRY_BLACK);
    rampQueue.push(LAUNDRY_RED);
    rampQueue.push(LAUNDRY_YELLOW);
    
    fullRouteStandard(L);
    scanLaundryBaskets();
    leaveLaundry();
    fullRouteStandard(S);
    finishProcedure();*/

    format(bt, "Mission Time: %  \r\n")%missionTimer.secElapsed();

    robot.stop(BRAKE);
    t.secDelay(1);
    format(bt, "\n\rENDING\n\r");
    bt.close();
}

//COLOR SENSOR TEST LOOPS
/*leftScanner.setNormalisation(false);
rightScanner.setNormalisation(true);
display.resetScreen();
while(true)
{
    colorspaceRGB left = leftScanner.getRGB();
    colorspaceRGB right = rightScanner.getRGB();
    tslp_tsk(10);
    display.format("L\nR:%  \nG:%  \nB:%  \nR\nR:%  \nG:%  \nB:%  \n")%left.red %left.green %left.blue %right.red %right.green %right.blue;
}*/
/*leftScanner.setNormalisation(false);
rightScanner.setNormalisation(false);
display.resetScreen();
while(true)
{
    colorspaceHSV left = leftScanner.getHSV();
    colorspaceHSV right = rightScanner.getHSV();
    tslp_tsk(10);
    display.format("L\nH:%  \nS:%  \nV:%  \nR\nH:%  \nS:%  \nV:%  \n")%left.hue %left.saturation %left.value %right.hue %right.saturation %right.value;
}*/
//leftSensor.setFiltering(false);
//leftSensor.setNormalisation(true);
//rightSensor.setFiltering(false);
//rightSensor.setNormalisation(true);
/*while(!btnEnter.isPressed())
{
    colorspaceRGB rgbL = leftSensor.getRGB();
    colorspaceRGB rgbR = rightSensor.getRGB();
    format(bt, "LS: R:%  G:%  B:%  W:%  \n\rRS: R:%  G:%  B:%  W:%  \n\n\n\r")%rgbL.red %rgbL.green %rgbL.blue %rgbL.white %rgbR.red %rgbR.green %rgbR.blue %rgbR.white;
    tslp_tsk(10);
}
t.secDelay(1);
while(!btnEnter.isPressed())
{
    format(bt, "LS: Ref: %  \n\rRS: Ref: %  \n\n\n\r")%leftSensor.getReflected() %rightSensor.getReflected();
    tslp_tsk(10);
}
t.secDelay(1);
while(!btnEnter.isPressed())
{
    colorspaceHSV hsvL = leftSensor.getHSV();
    colorspaceHSV hsvR = rightSensor.getHSV();
    format(bt, "LS: H:%  S:%  V:%  \n\rRS: H:%  S:%  V:%  \n\n\n\r")%hsvL.hue %hsvL.saturation %hsvL.value %hsvR.hue %hsvR.saturation %hsvR.value;
    tslp_tsk(10);
}
t.secDelay(1);
while(!btnEnter.isPressed())
{
    format(bt, "LS: col:%  \n\rRS: col:%  \n\n\n\r")%static_cast<int>(leftSensor.getColor()) %static_cast<int>(rightSensor.getColor());
    tslp_tsk(10);
}*/

/*leftSensor.setRefCalParams(6, 44);
rightSensor.setRefCalParams(4, 45);

double minL[] = {17, 32, 7, 56};
double maxL[] = {140, 174, 81, 395};
double minR[] = {12, 20, 8, 42};
double maxR[] = {127, 142, 83, 351};
leftSensor.setRgbCalParams(minL, maxL);
rightSensor.setRgbCalParams(minR, maxR);*/

/*color_hue cols[5];
cols[0] = {RED, 5, 10};
cols[1] = {RED, 330, 60};
cols[2] = {BLUE, 210, 80};
cols[3] = {GREEN, 120, 80};
cols[4] = {YELLOW, 40, 60};
leftSensor.setColorCalParams(cols, 5, 50, 18);
rightSensor.setColorCalParams(cols, 5, 50, 18);*/


//CALIBRATION

/*leftSensor.setNormalisation(false);
rightSensor.setNormalisation(false);
double minLeft[4] = {1000, 1000, 1000, 1000};
double maxLeft[4] = {0, 0, 0, 0};
double minRight[4] = {1000, 1000, 1000, 1000};
double maxRight[4] = {0, 0, 0, 0};
t.secDelay(1);
btnEnter.waitForPress();
minLeft[0] = leftSensor.getRGB().red;
minLeft[1] = leftSensor.getRGB().green;
minLeft[2] = leftSensor.getRGB().blue;
minLeft[3] = leftSensor.getRGB().white;
t.secDelay(1);
btnEnter.waitForPress();
maxLeft[0] = leftSensor.getRGB().red;
maxLeft[1] = leftSensor.getRGB().green;
maxLeft[2] = leftSensor.getRGB().blue;
maxLeft[3] = leftSensor.getRGB().white;
t.secDelay(1);
btnEnter.waitForPress();
minRight[0] = rightSensor.getRGB().red;
minRight[1] = rightSensor.getRGB().green;
minRight[2] = rightSensor.getRGB().blue;
minRight[3] = rightSensor.getRGB().white;
t.secDelay(1);
btnEnter.waitForPress();
maxRight[0] = rightSensor.getRGB().red;
maxRight[1] = rightSensor.getRGB().green;
maxRight[2] = rightSensor.getRGB().blue;
maxRight[3] = rightSensor.getRGB().white;

leftSensor.setRgbCalParams(minLeft, maxLeft);
rightSensor.setRgbCalParams(minRight, maxRight);*/
