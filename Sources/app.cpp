#include <cstdlib>

#include <vector>
#include <queue>
#include <list>
#include <map>
#include <cstdio>
#include <climits>
#include <cmath>

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
motor leftMotor(MotorPort::B, true, MotorType::MEDIUM);
motor rightMotor(MotorPort::C, false, MotorType::MEDIUM);
chassis robot(&leftMotor, &rightMotor, 6.24, 17, 0.1, 0.007);
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
    btnEnter.waitForClick();
    act_tsk(INIT_TASK);
    tslp_tsk(1);
}

bool grabberUsed = false;
bool startPicking = false;

void open_grabber_task(intptr_t unused)
{
    //INITIALIZATION OF GRABBER
    grabber.setMode(REGULATED);
    grabber.moveUnlimited(-1000, true);
    tslp_tsk(50);
    while(grabber.getCurrentSpeed() < -400)
    {
        grabber.moveUnlimited(-1000);
        tslp_tsk(1);
    }
    grabber.stop(BRAKE);
}

void init_task(intptr_t unused)
{
    //INITIALIZATION OF RAMP
    // act_tsk(OPEN_GRABBER_TASK);
    grabber.setMode(REGULATED);
    grabber.moveUnlimited(-1000, true);
    tslp_tsk(50);
    while(grabber.getCurrentSpeed() < -400)
    {
        grabber.moveUnlimited(-1000);
        tslp_tsk(1);
    }
    grabber.stop(BRAKE);
    ramp.setMode(REGULATED);
    ramp.moveUnlimited(-800, true);
    tslp_tsk(50);
    while(abs(ramp.getCurrentSpeed()) > 50)
    {
        ramp.moveUnlimited(-800);
        tslp_tsk(1);
    }
    ramp.stop(BRAKE);
}

void close_ramp_task(intptr_t unused)
{
    //CLOSES RAMP
    ramp.setMode(REGULATED);
    ramp.moveUnlimited(-800, true);
    tslp_tsk(50);
    while(abs(ramp.getCurrentSpeed()) > 50)
    {
        ramp.moveUnlimited(-800);
        tslp_tsk(1);
    }
    ramp.stop(BRAKE);
}

void water_grabber_task(intptr_t unused)
{
    //PICKS FIRST WATER OPENS GRABBER AND PICKS SECOND WATER WHEN COMMAND IS GIVEN via startPicking
    grabberUsed = true;
    grabber.setMode(REGULATED);
    grabber.moveUnlimited(800, true);
    tslp_tsk(50);
    while(grabber.getCurrentSpeed() > 350)
    {
        grabber.moveUnlimited(800);
        tslp_tsk(1);
    }
    grabber.stop(BRAKE_COAST);
    grabber.moveUnlimited(-1000, true);
    tslp_tsk(50);
    while(grabber.getCurrentSpeed() < -400)
    {
        grabber.moveUnlimited(-1000);
        tslp_tsk(1);
    }
    grabber.stop(BRAKE);
    grabberUsed = false;
}

void pick_block_task(intptr_t unused)
{
    // pickBlock();
    grabber.setMode(REGULATED);
    grabber.moveUnlimited(700, true);
    tslp_tsk(50);
    while(grabber.getCurrentSpeed() > 350)
    {
        grabber.moveUnlimited(700);
        tslp_tsk(1);
    }
    grabber.stop(BRAKE);
}

void empty_water_ramp_task(intptr_t unused)
{
    //LEAVES WATER ON TABLE FINAL STAGE
    emptyRampWaterStage2();
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
    // startProcedure();

    // fullRouteStandard(W);
    // pickWater();
    rampQueue.push(BOTTLE);
    rampQueue.push(BOTTLE);
    grabber.stop(BRAKE);

    // robot.setMode(CONTROLLED);
    // correctionBeforeMovement();
    // robot.setLinearAccelParams(150, 10, 20);
    // robot.straight(20, 5, NONE);
    // correctionOnTheMove();
    // robot.setLinearAccelParams(150, 20, 10);
    // robot.straight(35, 50, BRAKE);


    // robot.stop(BRAKE);
    // btnEnter.waitForClick();


    currentPos = FR;
    currentDirection = SOUTH;
    rooms[GREEN].setTask(WHITE);
    fullRouteStandard(GR);
    rooms[GREEN].executeAllActions();

    fullRouteStandard(RR);
    rooms[RED].setTask(WHITE);
    rooms[RED].executeAllActions();

    fullRouteStandard(CR);

    // fullRouteStandard(BR);
    // rooms[BLUE].executeAllActions();
    // fullRouteStandard(YR);
    // rooms[YELLOW].executeAllActions();

    robot.stop(BRAKE);



    // while(true) //BALL TEST (SUCCESS)
    // { 
    //     grabber.setMode(REGULATED);
    //     grabber.moveUnlimited(-700, true);
    //     tslp_tsk(50);
    //     while(grabber.getCurrentSpeed() < -350)
    //     {
    //         grabber.moveUnlimited(-700);
    //         tslp_tsk(1);
    //     }
    //     grabber.stop(BRAKE);   
    //     btnEnter.waitForClick();
    //     grabber.setMode(REGULATED);
    //     grabber.moveDegrees(400, 270, BRAKE);
    //     btnEnter.waitForClick();
    //     grabber.moveUnlimited(700, true);
    //     tslp_tsk(50);
    //     while(grabber.getCurrentSpeed() > 350)
    //     {
    //         grabber.moveUnlimited(700);
    //         tslp_tsk(1);
    //     }
    //     grabber.stop(BRAKE);
    //     btnEnter.waitForClick();
    // }
    
    // while(true)
    // {
    //     grabber.setStallTolerance(150, 10, 0.1);
    //     grabber.setSpeedLimiter(false);
    //     grabber.setMode(REGULATED);
    //     grabber.moveDegrees(-1400, 300, NONE);
    //     grabber.moveUntilStalled(-500, BRAKE);
    //     btnEnter.waitForClick(); 
    //     int tacho;
    //     int startSpeed = 1400;
    //     int endSpeed = 700;
    //     int distance = 300;
    //     int decelDistance = 100;
    //     grabber.moveUnlimited(startSpeed, true);
    //     while(tacho = grabber.getTachoCount() < (distance - decelDistance))
    //         grabber.moveUnlimited(startSpeed);
    //     grabber.resetTachoCount();
    //     while(tacho = grabber.getTachoCount() < decelDistance)
    //         grabber.moveUnlimited((decelDistance - tacho) / decelDistance * (startSpeed - endSpeed) + endSpeed);
    //     grabber.moveUntilStalled(endSpeed, BRAKE);
    // }

    // while(true)
    // {
    //     grabber.setMode(REGULATED);
    //     grabber.moveUnlimited(700, true);
    //     tslp_tsk(50);
    //     while(grabber.getCurrentSpeed() > 350)
    //     {
    //         grabber.moveUnlimited(700);
    //         tslp_tsk(1);
    //     }
    //     grabber.stop(BRAKE_COAST);
    //     grabber.moveUnlimited(-700, true);
    //     tslp_tsk(50);
    //     while(grabber.getCurrentSpeed() < -350)
    //     {
    //         grabber.moveUnlimited(-700);
    //         tslp_tsk(1);
    //     }
    //     grabber.stop(BRAKE);
    //     btnEnter.waitForClick();
    // }

    // btnEnter.waitForClick();

    // while(true)
    // {
    //     ramp.setMode(REGULATED);
    //     ramp.moveUnlimited(-800, true);
    //     tslp_tsk(50);
    //     while(abs(ramp.getCurrentSpeed()) > 50)
    //     {
    //         ramp.moveUnlimited(-800);   
    //         tslp_tsk(1);
    //     }
    //     ramp.stop(BRAKE);
    //     // btnEnter.waitForClick();
    //     // ramp.moveDegrees(400, 60, BRAKE);
    //     btnEnter.waitForClick();
    //     ramp.moveUnlimited(300, true);
    //     tslp_tsk(50);
    //     while(abs(ramp.getCurrentSpeed()) > 50)
    //     {
    //         ramp.moveUnlimited(300);
    //         tslp_tsk(1);
    //     }
    //     ramp.stop(BRAKE);
    //     btnEnter.waitForClick();
    // }

    // while(true)
    // {
    //     ramp.setMode(REGULATED);
    //     ramp.moveUnlimited(-800, true);
    //     tslp_tsk(50);
    //     while(abs(ramp.getCurrentSpeed()) > 50)
    //     {
    //         ramp.moveUnlimited(-800);
    //         tslp_tsk(1);
    //     }
    //     ramp.stop(BRAKE);
    //     btnEnter.waitForClick();
    //     ramp.moveDegrees(400, 60, BRAKE);
    //     btnEnter.waitForClick();
    //     ramp.moveUnlimited(600, true);
    //     tslp_tsk(50);
    //     while(abs(ramp.getCurrentSpeed()) > 50)
    //     {
    //         ramp.moveUnlimited(600);
    //         tslp_tsk(1);
    //     }
    //     ramp.stop(BRAKE);
    //     btnEnter.waitForClick();
    // }
    
    // grabber.stop(COAST);
    // ramp.stop(COAST);

    // leftScanner.setNormalisation(false);
    // rightScanner.setNormalisation(false);
    // display.resetScreen();
    // while(true)
    // {
    //     colorspaceRGB l = leftSensor.getRGB();
    //     colorspaceRGB r = rightSensor.getRGB();
    //     format(bt, "L: R: %  G: %  B: %  \nR: R: %  G: %  B: %  \n\n") %l.red %l.green %l.blue %r.red %r.green %r.blue;
    //     colorspaceHSV l2 = leftSensor.getHSV();
    //     colorspaceHSV r2 = rightSensor.getHSV();
    //     // format(bt, "L: H: %  S: %  V: %  \nR: H: %  S: %  V: %  \n\n") %l2.hue %l2.saturation %l2.value %r2.hue %r2.saturation %r2.value;
    //     //  display.format("L: %  \nR: %  \n\n\n") %static_cast<int>(leftSensor.getColor()) %static_cast<int>(rightSensor.getColor());
    //     tslp_tsk(10);
    // }

    // display.resetScreen();
    // while(true)
    // {
    //     display.format("L: %  \nR: %  \n\n\n") %static_cast<int>(leftSensor.getColor()) %static_cast<int>(rightSensor.getColor());
    //     tslp_tsk(10);
    // }

    // grabber.stop(COAST);
    // ramp.stop(COAST);
    // chassis robot2(&leftMotor, &rightMotor, 6.24, 17, 0.1, 0.01);
    // robot2.setMode(REGULATED);
    // double arcCenter = -5;
    // double angle = -3 * 180;
    // display.resetScreen();
    // while(true)
    // {
    //     // for(int i = 0; i < 4; i++)
    //     // {
    //     //     robot2.arc(robot2.cmToTacho(35), -90, -4, BRAKE);
    //     //     t.secDelay(0.5);
    //     // }
    //     // btnEnter.waitForClick();
    //     // for(int i = 0; i < 4; i++)
    //     // {
    //     //     robot2.arc(robot2.cmToTacho(35), -90, 4, BRAKE);
    //     //     t.secDelay(0.5);
    //     // }
    //     // btnEnter.waitForClick();
        
    //     // robot2.setMode(REGULATED);
    //     // robot2.tank(robot.cmToTacho(15), robot.cmToTacho(15), robot.cmToTacho(80));
    //     // btnEnter.waitForClick();
    //     // robot2.tank(robot.cmToTacho(15), robot.cmToTacho(15), robot.cmToTacho(-80));
    //     // btnEnter.waitForClick();

    //     // robot.setMode(CONTROLLED);
    //     // robot.setLinearAccelParams(150, 0, 0);
    //     // robot.straight(45, 80);
    //     // btnEnter.waitForClick();
    //     // // robot.setLinearAccelParams(150, -10, -10);
    //     // robot.straight(45, -80);
    //     // btnEnter.waitForClick();


    //     while(!btnEnter.isPressed())
    //     {
    //         BrickButton left(BrickButtons::LEFT);
    //         BrickButton right(BrickButtons::RIGHT);
    //         if(left.isPressed()) 
    //             angle--;
    //         else if(right.isPressed())
    //             angle++;
    //         display.format("%  \n%  \n%  \n%  \n") %angle %leftMotor.getTachoCount() %rightMotor.getTachoCount() %robot2.getAngle();
    //         tslp_tsk(100);
    //     }

    //     // robot2.setMode(CONTROLLED);
    //     // robot2.arc(35, angle, arcCenter);
    //     robot.setMode(REGULATED);
    //     robot.arc(robot.cmToTacho(35), angle, arcCenter);
    // }

    // robot2.stop(BRAKE);


    // grabber.stop();
    // ramp.stop();

    // leftScanner.setNormalisation(false);
    // rightScanner.setNormalisation(false);
    // display.resetScreen();
    // while(true)
    // {
    //     colorspaceRGB l = leftSensor.getRGB();
    //     colorspaceRGB r = rightSensor.getRGB();
    //     // format(bt, "L: R: %  G: %  B: %  \nR: R: %  G: %  B: %  \n\n") %l.red %l.green %l.blue %r.red %r.green %r.blue;
    //     colorspaceHSV l2 = leftSensor.getHSV();
    //     colorspaceHSV r2 = rightSensor.getHSV();
    //     // format(bt, "L: H: %  S: %  V: %  \nR: H: %  S: %  V: %  \n\n") %l2.hue %l2.saturation %l2.value %r2.hue %r2.saturation %r2.value;
    //      display.format("L: %  \nR: %  \n\n\n") %static_cast<int>(scanLaundryBlock(leftScanner)) %static_cast<int>(scanLaundryBlock(rightScanner));
    //     tslp_tsk(10);
    // }

    // robot.setMode(CONTROLLED);
    // setLifoNormalReg();
    // lifo.setAccelParams(150, 30, 35);
    // lifo.setPIDparams(1, 3, 200, 1);
    // lifo.distance(35, 12, NONE);
    // setLifoSlow();
    // lifo.setAccelParams(150, 35, 35);
    // lifo.lines(35, 1, NONE);
    // robot.setLinearAccelParams(150, 35, 15);
    // robot.straight(35, 15);
    // robot.setMode(REGULATED);
    // robot.arc(35, -90, 4);


    
    // while(true)
    // {
    //     resetLifo();
    //     setLifoLeftExtreme();
    //     lifo.distance(robot.cmToTacho(30), 15, NONE);
    //     setLifoSlow();
    //     setLifoLeft(true);
    //     lifo.setAccelParams(150, 20, 20);
    //     lifo.unlimited(20, true);
    //     while(rightSensor.getReflected() < 60)
    //         lifo.unlimited(20);
    //     lifo.stop(BRAKE);
    //     btnEnter.waitForClick();
    // }



//LIFO UNTIL ROOM ENTRANCE
    // resetLifo();
    // setLifoLeftExtreme();
    // lifo.distance(robot.cmToTacho(30), 10, NONE);
    // setLifoSlow();
    // setLifoLeft(true);
    // lifo.setAccelParams(150, 20, 20);
    // lifo.distance(20, 3, NONE);

//ENTER GREEN ROOM
    // robot.setMode(CONTROLLED);
    // robot.setLinearAccelParams(150, 20, 35);
    // robot.straight(35, 7, NONE);
    // robot.straightUnlim(35, true); 
    // while(!detectWhiteRoomBed(rightSensor))
    //     robot.straightUnlim(35);
    // robot.setLinearAccelParams(150, 35, 15);
    // robot.straight(20, 6.7, BRAKE);   //ADD 0.7cm if room is NOT red

//LAUNDRY + BALL GREEN
    // act_tsk(OPEN_GRABBER_TASK);
    // tslp_tsk(1);
    // robot.setMode(REGULATED);
    // robot.arc(25, -45, -8.5, NONE);    

    // robot.arc(30, -45, -3, BRAKE);

    // robot.setMode(CONTROLLED);
    // robot.setLinearAccelParams(150, 15, 35);
    // robot.straight(35, 5, NONE);    
    // act_tsk(WATER_GRABBER_TASK);   
    // tslp_tsk(1);
    // robot.setLinearAccelParams(150, 35, 20);
    // robot.straight(30, 12, NONE);

    // robot.setMode(REGULATED);
    // robot.arc(30, 91, -3.5);
    // while(grabberUsed)
    //     tslp_tsk(10); 

    // robot.setMode(CONTROLLED);
    // robot.setLinearAccelParams(150, 20, 20);
    // robot.straight(25, 7, BRAKE);
    // grabber.moveDegrees(600, 210, NONE, true);
    // grabber.moveDegrees(400, 100, BRAKE, false);
    // robot.setMode(REGULATED);
    // robot.arc(30, 82, -5, NONE);

    // robot.setMode(CONTROLLED);
    // robot.setLinearAccelParams(150, 15, 20);
    // robot.straight(45, 17.5, NONE);
    // act_tsk(PICK_BLOCK_TASK);
    // tslp_tsk(1);
    // robot.setLinearAccelParams(150, 20, 15);
    // robot.straight(20, 1, BRAKE);
    // robot.setLinearAccelParams(150, -15, -15);
    // robot.straight(45, -8, BRAKE);
    // robot.setAngularAccelParams(600, -200, -200);
    // robot.turn(300, -103, BRAKE);
    // robot.setLinearAccelParams(150, 15, 35);
    // robot.straight(40, 19.5, NONE);



//NO LAUNDRY + BALL GREEN
    // act_tsk(OPEN_GRABBER_TASK);
    // tslp_tsk(1);
    // robot.setMode(REGULATED);
    // robot.arc(25, 50, 5, BRAKE);    
    // while(grabberUsed)
    //     tslp_tsk(10); 
    // robot.setMode(CONTROLLED);
    // robot.setLinearAccelParams(150, 20, 20);
    // robot.straight(25, 5, BRAKE);
    // grabber.moveDegrees(600, 210, NONE, true);
    // grabber.moveDegrees(400, 100, BRAKE, false);
    // robot.setMode(REGULATED);
    // robot.arc(30, -115, 2, BRAKE);
    // robot.setLinearAccelParams(150, 15, 20);
    // robot.straight(45, 15.5, NONE);
    // act_tsk(PICK_BLOCK_TASK);
    // tslp_tsk(1);
    // robot.setLinearAccelParams(150, 20, 15);
    // robot.straight(20, 1, BRAKE);
    // robot.setLinearAccelParams(150, -15, -15);
    // robot.straight(45, -9.5, BRAKE);
    // robot.setAngularAccelParams(600, -200, -200);
    // robot.turn(300, -115, BRAKE);
    // robot.setLinearAccelParams(150, 15, 35);
    // robot.straight(40, 17, NONE);

//NO LAUNDRY + WATER GREEN
    // robot.setMode(REGULATED);
    // robot.arc(25, -45, -8.5, NONE);    

    // emptyRampWaterStage1(false);
    // robot.arc(30, -45, -3, BRAKE);
    // emptyRampWaterStage2();

    // robot.setMode(CONTROLLED);
    // robot.setLinearAccelParams(150, 15, 15);
    // robot.straight(35, 3, NONE);    

    // robot.setMode(REGULATED);
    // robot.arc(35, 85, 3.5, NONE);
    // act_tsk(CLOSE_RAMP_TASK);
    // tslp_tsk(1);
    // robot.arcUnlim(35, 3.5, FORWARD, true);
    // while(rightSensor.getReflected() < 60)
    //     robot.arcUnlim(35, 3.5, FORWARD, false);


//LAUNDRY + WATER GREEN
    // act_tsk(OPEN_GRABBER_TASK);
    // tslp_tsk(1);
    // robot.setMode(REGULATED);
    // robot.arc(25, -45, -8.5, NONE);    

    // emptyRampWaterStage1(false);
    // robot.arc(30, -45, -3, BRAKE);
    // emptyRampWaterStage2();

    // robot.setMode(CONTROLLED);
    // robot.setLinearAccelParams(150, 15, 35);
    // robot.straight(35, 5, NONE);    
    // act_tsk(CLOSE_RAMP_TASK);
    // tslp_tsk(1);
    // act_tsk(PICK_BLOCK_TASK);   
    // tslp_tsk(1);
    // robot.setLinearAccelParams(150, 35, 15);
    // robot.straight(30, 10, BRAKE);

    // robot.setLinearAccelParams(150, -15, -15);
    // robot.straight(35, -10, BRAKE);

    // robot.setMode(REGULATED);
    // robot.arc(35, 90, 3.5, NONE);
    // robot.arcUnlim(35, 3.5, FORWARD, true);
    // while(rightSensor.getReflected() < 60)
    //     robot.arcUnlim(35, 3.5, FORWARD, false);


//GR -> RR
    // resetLifo();
    // setLifoRightExtreme();
    // lifo.distance(robot.cmToTacho(30), 10, NONE);
    // setLifoRight();
    // while(!leftSensor.getLineDetected())
    //     executeLifoRightUnlim(robot.cmToTacho(30));
    // robot.resetPosition();
    // while(robot.getPosition() < 5)
    //     executeLifoRightUnlim(robot.cmToTacho(30));

    // resetLifo();
    // setLifoRightExtreme();
    // lifo.distance(robot.cmToTacho(30), 10, NONE);
    // setLifoSlow();
    // setLifoRight(true);
    // lifo.setAccelParams(150, 20, 20);
    // lifo.distance(20, 3, NONE);

    
// ENTER RED ROOM
    // robot.setMode(CONTROLLED);
    // robot.setLinearAccelParams(150, 20, 35);
    // robot.straight(35, 7, NONE);
    // robot.straightUnlim(35, true); 
    // while(!detectWhiteRoomBed(rightSensor))
    //     robot.straightUnlim(35);
    // robot.setLinearAccelParams(150, 35, 15);
    // robot.straight(20, 6, BRAKE);   //ADD 0.7cm if room is NOT red



//WATER + LAUNDRY RED
    // act_tsk(OPEN_GRABBER_TASK);
    // tslp_tsk(1);
    // robot.setMode(REGULATED);
    // robot.arc(25, -45, 8.5, NONE);    

    // emptyRampWaterStage1(false);
    // robot.arc(30, -45, 3, BRAKE);
    // emptyRampWaterStage2();

    // robot.setMode(CONTROLLED);
    // robot.setLinearAccelParams(150, 15, 35);
    // robot.straight(35, 5, NONE);    
    // act_tsk(CLOSE_RAMP_TASK);
    // tslp_tsk(1);
    // act_tsk(PICK_BLOCK_TASK);   
    // tslp_tsk(1);
    // robot.setLinearAccelParams(150, 35, 15);
    // robot.straight(30, 10, BRAKE);

    // robot.setLinearAccelParams(150, -15, -15);
    // robot.straight(35, -10, BRAKE);

    // robot.setMode(REGULATED);
    // robot.arc(35, 90, -3.5, NONE);
    // robot.arcUnlim(35, -3.5, FORWARD, true);
    // while(rightSensor.getReflected() < 60)
    //     robot.arcUnlim(35, -3.5, FORWARD, false);

//NO LAUNDRY + WATER RED
    // robot.setMode(REGULATED);
    // robot.arc(25, -45, 8.5, NONE);    

    // emptyRampWaterStage1(false);
    // robot.arc(30, -45, 3, BRAKE);
    // emptyRampWaterStage2();

    // robot.setMode(CONTROLLED);
    // robot.setLinearAccelParams(150, 15, 15);
    // robot.straight(35, 3, NONE);    
    
    // robot.setMode(REGULATED);
    // robot.arc(35, 85, -3.5, NONE);
    // act_tsk(CLOSE_RAMP_TASK);
    // tslp_tsk(1);
    // robot.arcUnlim(35, -3.5, FORWARD, true);
    // while(rightSensor.getReflected() < 60)
    //     robot.arcUnlim(35, -3.5, FORWARD, false);

// NO LAUNDRY + BALL RED
    // act_tsk(OPEN_GRABBER_TASK);
    // tslp_tsk(1);
    // robot.setMode(REGULATED);
    // robot.arc(25, 50, -5, BRAKE);    
    // while(grabberUsed)
    //     tslp_tsk(10); 
    // robot.setMode(CONTROLLED);
    // robot.setLinearAccelParams(150, 20, 20);
    // robot.straight(25, 5, BRAKE);
    // grabber.moveDegrees(600, 210, NONE, true);
    // grabber.moveDegrees(400, 100, BRAKE, false);
    // robot.setMode(REGULATED);
    // robot.arc(30, -115, -2, BRAKE);
    // robot.setLinearAccelParams(150, 15, 20);
    // robot.straight(45, 15.5, NONE);
    // act_tsk(PICK_BLOCK_TASK);
    // tslp_tsk(1);
    // robot.setLinearAccelParams(150, 20, 15);
    // robot.straight(20, 1, BRAKE);
    // robot.setLinearAccelParams(150, -15, -15);
    // robot.straight(45, -8.5, BRAKE);
    // robot.setAngularAccelParams(600, 200, 200);
    // robot.turn(300, 128, BRAKE);
    // robot.setLinearAccelParams(150, 15, 35);
    // robot.straight(40, 17, NONE);

// LAUNDRY + BALL RED
    // act_tsk(OPEN_GRABBER_TASK);
    // tslp_tsk(1);
    // robot.setMode(REGULATED);
    // robot.arc(25, -45, 8.5, NONE);    

    // robot.arc(30, -45, 3, BRAKE);

    // robot.setMode(CONTROLLED);
    // robot.setLinearAccelParams(150, 15, 35);
    // robot.straight(35, 5, NONE);    
    // act_tsk(WATER_GRABBER_TASK);   
    // tslp_tsk(1);
    // robot.setLinearAccelParams(150, 35, 20);
    // robot.straight(30, 12, NONE);

    // robot.setMode(REGULATED);
    // robot.arc(30, 95, 3.5);
    // while(grabberUsed)
    //     tslp_tsk(10); 

    // robot.setMode(CONTROLLED);
    // robot.setLinearAccelParams(150, 20, 20);
    // robot.straight(25, 7, BRAKE);
    // grabber.moveDegrees(600, 210, NONE, true);
    // grabber.moveDegrees(400, 100, BRAKE, false);
    // robot.setMode(REGULATED);
    // robot.arc(30, 82, 5, NONE);

    // robot.setMode(CONTROLLED);
    // robot.setLinearAccelParams(150, 15, 20);
    // robot.straight(45, 17.5, NONE);
    // act_tsk(PICK_BLOCK_TASK);
    // tslp_tsk(1);
    // robot.setLinearAccelParams(150, 20, 15);
    // robot.straight(20, 1, BRAKE);
    // robot.setLinearAccelParams(150, -15, -15);
    // robot.straight(45, -8, BRAKE);
    // robot.setAngularAccelParams(600, 200, 200);
    // robot.turn(300, 110, BRAKE);
    // robot.setLinearAccelParams(150, 15, 35);
    // robot.straight(40, 19.5, NONE);


//RR -> FR (placeholder for testing)
    // resetLifo();
    // setLifoLeftExtreme();
    // lifo.distance(robot.cmToTacho(30), 10, NONE);
    // setLifoLeft();
    // while(!rightSensor.getLineDetected())
    //     executeLifoLeftUnlim(robot.cmToTacho(30));


    // robot.setMode(CONTROLLED);
    // robot.setAngularAccelParams(600, -200, -200);
    // robot.turn(300, -45, BRAKE);
    // robot.setLinearAccelParams(150, 10, 30);
    // robot.straight(45, 15, NONE);

    // resetLifo();
    // lifo.setPIDparams(KP * 1.5, slowKI * 0.7, KD*2, 1);
    // lifo.distance(robot.cmToTacho(30), 10, NONE);
    // setLifoSlow();
    // lifo.setAccelParams(150, 20, 20);
    // lifo.distance(20, 3, NONE);
    // lifo.lines(20, 1, NONE);

    // robot.setLinearAccelParams(150, 20, 25);
    // robot.straight(25, 5, COAST);
    // act_tsk(WATER_GRABBER_TASK);
    // tslp_tsk(1);
    // robot.setLinearAccelParams(150, 25, 25);
    // robot.straight(25, 2, BRAKE);
    // while(grabber.getTachoCount() < 200) 
    //     tslp_tsk(10);

    // robot.setMode(REGULATED);
    // robot.arc(35, -34, -8.5);
    // while(grabberUsed)
    //     tslp_tsk(10);

    // robot.setMode(CONTROLLED);
    // robot.setLinearAccelParams(150, 10, 30);
    // robot.straight(30, 10, COAST);
    // act_tsk(PICK_BLOCK_TASK);
    // tslp_tsk(1);
    // robot.setLinearAccelParams(150, 30, 30);
    // robot.straight(30, 3, BRAKE);
    // while(grabber.getTachoCount() < 200) 
    //     tslp_tsk(10);

    // robot.arc(35, 90, 3, NONE);

    // robot.setMode(CONTROLLED);
    // robot.setLinearAccelParams(150, 30, 30);
    // robot.straight(30, 1, NONE);
    // robot.straightUnlim(30, true);
    // while(rightSensor.getReflected() > 80) tslp_tsk(1);
    // while(leftSensor.getReflected() > 80) tslp_tsk(1);
    // resetLifo();
    // lifo.setPIDparams(KP*1.5, slowKI * 0.7, KD*2, 1);
    // lifo.distance(robot.cmToTacho(30), 6, NONE);
    // setLifoSlow();
    // lifo.setAccelParams(150, 30, 30);
    // lifo.distance(30, 4, NONE);
    // lifo.lines(30, 1, NONE);

    // robot.setMode(REGULATED);
    // robot.arc(35, 35, 15, NONE);
    // robot.arc(40, 15, 30, NONE);
    // robot.arcUnlim(40, 30, FORWARD, true);
    // colors current = BLACK;
    // map<colors, int> appearances;
    // while(robot.getAngle() < 40)
    // {
    //     if((current = scanCodeBlock(leftScanner)) != BLACK)
    //     {
    //         appearances[current]++;
    //     }
    //     robot.arcUnlim(40, 30, FORWARD, false);
    // }
    // int maxCount = 0;
    // for(auto x: appearances)
    // {
    //     if(x.second > maxCount)
    //     {
    //         maxCount = x.second;
    //         current = x.first;    
    //     }
    // }
    // rooms[RED].setTask(current);
    // display.resetScreen();
    // display.format("%  \n")%static_cast<int>(current);

    // robot.setMode(CONTROLLED);
    // robot.setLinearAccelParams(150, 30, 35);
    // robot.straightUnlim(35, true);
    // leftSensor.getReflected();
    // while(!leftSensor.getLineDetected())
    // {
    //     leftSensor.getReflected();
    //     robot.straightUnlim(35);
    // }
    // robot.resetPosition();
    // while(robot.getPosition() < 1) robot.straightUnlim(35);
    // leftSensor.getReflected();
    // while(leftSensor.getLineDetected())
    // {
    //     leftSensor.getReflected();
    //     robot.straightUnlim(35);
    // }
    // robot.resetPosition();
    // while(robot.getPosition() < 1) robot.straightUnlim(35);
    // while(leftSensor.getReflected() > 80) robot.straightUnlim(35);
    // robot.setLinearAccelParams(150, 35, 10);
    // robot.straight(35, 8, NONE);

    // robot.setMode(REGULATED);
    // robot.arc(35, -75, 3.5, NONE); 
    // robot.arcUnlim(35, 3.5, BACKWARD, true);
    // while(leftSensor.getReflected() < 50)
    //     robot.arcUnlim(35, 3.5, BACKWARD, false);

    // resetLifo();
    // setLifoLeftExtreme();
    // lifo.distance(robot.cmToTacho(30), 5, NONE);
    // setLifoLeft();
    // while(rightSensor.getReflected() < 60)
    //     executeLifoLeftUnlim(robot.cmToTacho(30));

    // current = BLACK;
    // appearances.clear();
    // robot.resetPosition();
    // while(robot.getPosition() < 8)
    // {
    //     if((current = scanCodeBlock(rightScanner)) != BLACK)
    //     {
    //         appearances[current]++;
    //     }
    //     executeLifoLeftUnlim(robot.cmToTacho(30));
    // }
    // maxCount = 0;
    // for(auto x: appearances)
    // {
    //     if(x.second > maxCount)
    //     {
    //         maxCount = x.second;
    //         current = x.first;    
    //     }
    // }
    // rooms[GREEN].setTask(current);
    // display.format("%  \n")%static_cast<int>(current);


    // resetLifo();
    // setLifoLeftExtreme();
    // lifo.distance(robot.cmToTacho(30), 8, NONE);
    
    // lifo1WhiteLineLeftSlow(35, 2, 35, NONE);

    // robot.setMode(CONTROLLED);
    // robot.setLinearAccelParams(150, 35, 10);
    // robot.straight(35, 7, NONE);

    // robot.setMode(REGULATED);
    // robot.arc(35, -80, -4.5, NONE);
    // robot.arcUnlim(35, -4.5, BACKWARD, true);
    // while(leftSensor.getReflected() > 60)
    //     robot.arcUnlim(35, -4.5, BACKWARD); 


    // FR_GR(SOUTH);
    // rooms[GREEN].executeAllActions();
    // GR_RR(NORTH);
    // rooms[RED].executeAllActions();

    //  resetLifo();
    // setLifoLeftExtreme();
    // lifo.distance(robot.cmToTacho(30), 10, NONE);
    // setLifoLeft();
    // while(!rightSensor.getLineDetected())
    //     executeLifoLeftUnlim(robot.cmToTacho(30));


//GREEN ROOM LAUNDRY + WATER START
    // emptyRampWaterStage1(false);
    // act_tsk(OPEN_GRABBER_TASK);
    // tslp_tsk(1);
    // robot.setLinearAccelParams(150, -10, -20);
    // robot.straight(-20, 3, NONE);
    // robot.arc(35, -95, -5, BRAKE);
    // // robot.setLinearAccelParams(150, -10, -10);
    // // robot.straight(20, -2);
    // emptyRampWaterStage2();
    // robot.setLinearAccelParams(150, 10, 35);
    // robot.straight(35, 5, NONE);    
    // robot.setLinearAccelParams(150, 35, 35);
    // act_tsk(CLOSE_RAMP_TASK);
    // tslp_tsk(1);
    // robot.straight(35, 3, NONE);
    // act_tsk(PICK_BLOCK_TASK);   
    // tslp_tsk(1);
    // robot.setLinearAccelParams(150, 35, 10);
    // robot.straight(35, 8, BRAKE);

    // robot.setLinearAccelParams(150, -10, -10);
    // robot.straight(35, -12);

    // robot.arc(35, 80, 4, NONE);
    // robot.arcUnlim(35, 4, FORWARD, true);
    // while(rightSensor.getReflected() < 65)
    //     robot.arcUnlim(35, 4, FORWARD);   
//GREEN ROOM LAUNDRY + WATER END


// GREEN ROOM WATER + NO LAUNDRY START
    // robot.setMode(REGULATED);
    // emptyRampWaterStage1(false);
    // robot.arc(robot.cmToTacho(35), -88, -4, NONE);
    // robot.setMode(CONTROLLED);
    // robot.setLinearAccelParams(150, -10, -10);
    // robot.straight(20, -1.5);
    // emptyRampWaterStage2();
    // robot.setMode(CONTROLLED);
    // robot.setLinearAccelParams(150, 10, 20);
    // robot.straight(25, 4.5, NONE);
    // robot.setMode(REGULATED);
    // robot.arc(robot.cmToTacho(35), 90, 2.5, NONE);
    // act_tsk(CLOSE_RAMP_TASK);
    // tslp_tsk(1);
    // robot.setMode(CONTROLLED);
    // robot.setLinearAccelParams(150, 25, 35);
    // robot.straight(35, 4, NONE);
//GREEN ROOM WATER + NO LAUNDRY END

//GREEN ROOM BALL + LAUNDRY START
    // robot.setMode(REGULATED);
    // act_tsk(OPEN_GRABBER_TASK);
    // tslp_tsk(1);
    // robot.arc(robot.cmToTacho(35), -88, -4, NONE);
    // robot.setMode(CONTROLLED);
    // robot.setLinearAccelParams(150, -10, -10);
    // robot.straight(20, -1.5);
    // robot.setMode(CONTROLLED);
    // robot.setLinearAccelParams(150, 10, 35);
    // robot.straight(35, 5, NONE);    
    // robot.setLinearAccelParams(150, 35, 35);
    // robot.straight(35, 3, NONE);
    // act_tsk(WATER_GRABBER_TASK);   
    // tslp_tsk(1);
    // robot.setLinearAccelParams(150, 35, 10);
    // robot.straight(35, 9, BRAKE);
    // robot.setMode(REGULATED);
    // robot.arc(robot.cmToTacho(25), 85, -4);
    // while(grabberUsed)
    //     tslp_tsk(10); 
    // robot.setMode(CONTROLLED);
    // robot.setLinearAccelParams(150, 25, 25);
    // robot.straight(25, 7, BRAKE);
    // grabber.moveDegrees(600, 150, NONE, true);
    // grabber.moveDegrees(400, 150, BRAKE, false);
    // robot.setMode(REGULATED);
    // robot.arc(robot.cmToTacho(30), 78, -4, NONE);
    // robot.setMode(CONTROLLED);
    // robot.setLinearAccelParams(150, 25, 20);
    // robot.straight(40, 18, NONE);
    // act_tsk(PICK_BLOCK_TASK);
    // tslp_tsk(1);
    // robot.setLinearAccelParams(150, 20, 15);
    // robot.straight(20, 2, BRAKE);
    // robot.setLinearAccelParams(150, -15, -15);
    // robot.straight(40, -11, BRAKE);
    // robot.setMode(REGULATED);
    // robot.arc(robot.cmToTacho(30), 95, -4, NONE);
    // robot.setMode(CONTROLLED);
    // robot.setLinearAccelParams(150, 25, 35);
    // robot.straight(40, 17, NONE);
//GREEN ROOM BALL + LAUNDRY END


//GREEN ROOM BALL + NO LAUNDRY START
    // robot.setMode(REGULATED);
    // act_tsk(OPEN_GRABBER_TASK);
    // tslp_tsk(1);
    // robot.arc(robot.cmToTacho(25), 45, 9);
    // while(grabberUsed)
    //     tslp_tsk(10); 
    // robot.setMode(CONTROLLED);
    // robot.setLinearAccelParams(150, 20, 20);
    // robot.straight(25, 6, BRAKE);
    // grabber.moveDegrees(600, 210, NONE, true);
    // grabber.moveDegrees(400, 100, BRAKE, false);
    // robot.setMode(REGULATED);
    // robot.arc(robot.cmToTacho(30), -107, 1.5);
    // robot.setMode(CONTROLLED);
    // robot.setLinearAccelParams(150, 15, 20);
    // robot.straight(40, 15.5, NONE);
    // act_tsk(PICK_BLOCK_TASK);
    // tslp_tsk(1);
    // robot.setLinearAccelParams(150, 20, 15);
    // robot.straight(20, 2, BRAKE);
    // robot.setLinearAccelParams(150, -15, -15);
    // robot.straight(40, -8.5, BRAKE);
    // robot.setAngularAccelParams(600, -200, -200);
    // robot.turn(300, -110, BRAKE);
    // robot.setLinearAccelParams(150, 15, 35);
    // robot.straight(40, 17, NONE);
//GREEN ROOM BALL + NO LAUNDRY END


    // grabber.stop(COAST);
    // setLifoRightExtreme();
    // lifo.setAccelParams(150, 35, 25);
    // lifo.distance(25, 7, NONE); 
    // lifo1WhiteLineRightSlow(25, 6, 25, NONE);
    // robot.setMode(CONTROLLED);
    // robot.setLinearAccelParams(150, 25, 25);
    // robot.straight(25, 8, NONE);
    // setLifoRightExtreme();
    // lifo.setAccelParams(150, 25, 30);
    // lifo.distance(25, 10, NONE); 
    // setLifoRight(true);
    // lifo.setAccelParams(150, 30, 35);
    // lifo.distance(35, 5, NONE);

    // rooms[RED].enterRoom();


// //RED ROOM LAUNDRY + WATER START
    // emptyRampWaterStage1(false);
    // act_tsk(OPEN_GRABBER_TASK);
    // tslp_tsk(1);
    // robot.arc(35, -100, 3, NONE);
    // robot.setLinearAccelParams(150, -10, -10);
    // robot.straight(20, -2);
    // emptyRampWaterStage2();
    // robot.setLinearAccelParams(150, 10, 35);
    // robot.straight(35, 5, NONE);    
    // robot.setLinearAccelParams(150, 35, 35);
    // act_tsk(CLOSE_RAMP_TASK);
    // tslp_tsk(1);
    // robot.straight(35, 3, NONE);
    // act_tsk(PICK_BLOCK_TASK);   
    // tslp_tsk(1);
    // robot.setLinearAccelParams(150, 35, 10);
    // robot.straight(35, 8, BRAKE);

    // robot.setLinearAccelParams(150, -10, -10);
    // robot.straight(35, -12);

    // robot.arc(35, 80, 4, NONE);
    // robot.arcUnlim(35, 4, FORWARD, true);
    // while(rightSensor.getReflected() < 65)
    //     robot.arcUnlim(35, 4, FORWARD); 



//     robot.setMode(REGULATED);
//     emptyRampWaterStage1(false);
//     act_tsk(OPEN_GRABBER_TASK);
//     tslp_tsk(1);
//     robot.arc(robot.cmToTacho(35), -88, 4, NONE);
//     robot.setMode(CONTROLLED);
//     robot.setLinearAccelParams(150, -10, -10);
//     robot.straight(20, -1.5);
//     emptyRampWaterStage2();
//     robot.setMode(CONTROLLED);
//     robot.setLinearAccelParams(150, 10, 35);
//     robot.straight(35, 5, NONE);    
//     robot.setLinearAccelParams(150, 35, 35);
//     act_tsk(CLOSE_RAMP_TASK);
//     tslp_tsk(1);
//     robot.straight(35, 3, NONE);
//     act_tsk(PICK_BLOCK_TASK);   
//     tslp_tsk(1);
//     robot.setLinearAccelParams(150, 35, 10);
//     robot.straight(35, 8, BRAKE);

//     robot.setLinearAccelParams(150, -10, -10);
//     robot.straight(35, -12);
    
//     robot.setMode(REGULATED);
//     robot.arc(robot.cmToTacho(35), 85, -5, NONE); 
//     robot.arcUnlim(robot.cmToTacho(35), -5, FORWARD, true);
//     while(leftSensor.getReflected() < 65)
//         robot.arcUnlim(robot.cmToTacho(35), -5, FORWARD);  
// //RED ROOM LAUNDRY + WATER END

//     grabber.stop(COAST);
//     setLifoLeftExtreme();
//     lifo.setAccelParams(150, 35, 25);
//     lifo.distance(25, 7, NONE); 
//     lifo1WhiteLineLeftSlow(25, 6, 25, NONE);
//     robot.setMode(REGULATED);
//     robot.arc(robot.cmToTacho(35), 25, 9, NONE); 
//     robot.arc(robot.cmToTacho(35), 60, 19.5, NONE);
//     robot.arcUnlim(robot.cmToTacho(35), 19.5, FORWARD, true);
//     while(rightSensor.getReflected() < 60)
//         robot.arcUnlim(robot.cmToTacho(35), 19.5, FORWARD);   

//     setLifoRightExtreme();
//     lifo.setAccelParams(150, 20, 35);
//     lifo.distance(20, 15, NONE); 
//     setLifoRight(true);
//     lifo.setAccelParams(150, 35, 45);
//     lifo1WhiteLineRightSlow(35, 5, 45, NONE);
//     robot.setLinearAccelParams(150, 45, 30);
//     robot.straight(45, 90, NONE);
//     setLifoRightExtreme();
//     lifo.initializeMotionMode(CONTROLLED);
//     lifo.setAccelParams(150, 30, 30);
//     lifo.unlimited(30, true);
//     while(robot.getPosition() < 8)
//         executeLifoRightUnlim(30);
//     while(leftSensor.getReflected() < 60)
//         executeLifoRightUnlim(30);







    // robot.setMode(REGULATED);
    // // act_tsk(OPEN_GRABBER_TASK);
    // tslp_tsk(1);
    // robot.arc(robot.cmToTacho(35), -92, 4, NONE);
    // robot.setMode(CONTROLLED);
    // robot.setLinearAccelParams(150, -10, -10);
    // ramp.moveDegrees(350, 110, BRAKE, false);
    // robot.straight(20, -1.5);
    // while(ramp.getTachoCount() < 110) tslp_tsk(1);
    // // robot.setMode(CONTROLLED);
    // // robot.setLinearAccelParams(150, 10, 35);
    // // robot.straight(35, 3, NONE);
    // // robot.setLinearAccelParams(150, 35, 35);
    // // act_tsk(CLOSE_RAMP_TASK);
    // // tslp_tsk(1);
    // // robot.straight(35, 5, NONE);
    // // act_tsk(PICK_BLOCK_TASK);   
    // // tslp_tsk(1);
    // // robot.setLinearAccelParams(150, 35, 10);
    // // robot.straight(35, 6, BRAKE);

    // // robot.setLinearAccelParams(150, -10, -10);
    // // robot.straight(35, -11.5);
    
    // // robot.setMode(REGULATED);
    // // robot.arc(robot.cmToTacho(35), 85, -5, NONE); 
    // // robot.arcUnlim(robot.cmToTacho(35), -5, FORWARD, true);
    // // while(leftSensor.getReflected() < 65)
    // //     robot.arcUnlim(robot.cmToTacho(35), -5, FORWARD); 

    // robot.setMode(CONTROLLED);
    // robot.setLinearAccelParams(150, 10, 25);
    // robot.straight(25, 5, NONE);
    // act_tsk(CLOSE_RAMP_TASK);
    // tslp_tsk(1);
    // robot.setMode(REGULATED);
    // robot.arc(robot.cmToTacho(30), 87, -2, NONE);
    // robot.setLinearAccelParams(150, 10, 35);
    // robot.straight(35, 5, NONE);


    // setLifoLeftExtreme();
    // lifo.setAccelParams(150, 35, 35);
    // lifo.distance(35, 7, NONE); 
    // lifo1WhiteLineLeftSlow(35, 8, 35, NONE);


    

    // robot.setMode(REGULATED);
    // robot.arc(robot.cmToTacho(35), 25, 9, NONE); 
    // robot.arc(robot.cmToTacho(35), 70, 17, NONE); 
    // robot.arcUnlim(robot.cmToTacho(35), 17, FORWARD, true);
    // while(rightSensor.getReflected() < 60)
    //     robot.arcUnlim(robot.cmToTacho(35), 17, FORWARD);  

    // t.secDelay(0.2);
    // while(true)
    // {
    //     btnEnter.waitForClick();
    //     grabber.moveDegrees(500, 90, NONE);
    //     btnEnter.waitForClick();
    //     grabber.moveUntilStalled(-200, BRAKE);
    // }

    // lifo1WhiteLineRightSlow(30, 12, 30, NONE);
    // robot.setLinearAccelParams(200, 30, 30);
    // robot.straight(30, 8, NONE);
    // lifo.initializeMotionMode(CONTROLLED);
    // lifo.setDoubleFollowMode("70", "SR");
    // lifo.setAlignMode(true);
    // lifo.setPIDparams(slowKP*3, slowKI*1.8, slowKD*5, 1);
    // lifo.setAccelParams(200, 30, 20);
    // lifo.distance(20, 7, NONE);
    // lifo.setPIDparams(slowKP*1.7, slowKI*1.7, slowKD*2, 1);
    // lifo.distance(20, 8, NONE);

    // rooms[RED].setTask(WHITE);
    // rooms[RED].enterRoom();

    
    
    // robot.setAngularAccelParams(1000, -150, 0);
    // robot.turn(300, -25, NONE);
    // robot.setLinearAccelParams(100, -10, -10);
    // robot.straight(-45, 4.5, NONE);
    // act_tsk(OPEN_GRABBER_TASK);
    // robot.turn(300, -75, NONE);
    // robot.straight(-45, 3.5, BRAKE);

    // ramp.moveDegrees(300, 110, BRAKE);

    // robot.setLinearAccelParams(100, 10, 20);
    // robot.straight(20, 3, NONE);
    // act_tsk(CLOSE_RAMP_TASK);
    // // robot.setLinearAccelParams(100, 20, 10);
    // // robot.straight(20, 3, NONE);

    // robot.setLinearAccelParams(100, 20, 10);
    // robot.straight(45, 6);
    // grabber.moveDegrees(250, 100, NONE);
    // grabber.moveDegrees(60, 50, BRAKE, false);
    // robot.setLinearAccelParams(100, 0, 0);
    // robot.straight(-45, 3);
    // robot.setAngularAccelParams(1000, -150, 0);
    // robot.turn(300, -90, NONE);
    // robot.setLinearAccelParams(100, 10, 30);
    // robot.straight(30, 4, NONE);

    

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

    // robot.setMode(CONTROLLED);
    // robot.setAngularAccelParams(600, 150, 100);
    // robot.turn(300, 90, BRAKE_COAST);
    // robot.setLinearAccelParams(150, 20, 15);
    // robot.straight(45, 15, BRAKE_COAST);
    // robot.setAngularAccelParams(600, -150, -100);
    // robot.turn(300, -90, BRAKE_COAST);
    // robot.setLinearAccelParams(150, -20, -15);
    // robot.straight(45, -15, BRAKE_COAST);
    // robot.setLinearAccelParams(200, 20, 30);
    // robot.straight(45, 15, NONE);
    
    // lifo.setPIDparams(slowKP, slowKI, slowKD, 1);
    // lifo.setAlignMode();
    // lifo.setAccelParams(250, 20, 20);
    // lifo.distance(20, 5, NONE);
    // lifo.distance(45, 60, NONE);
    // lifo.lines(20, 1, COAST);
    // robot.setMode(REGULATED);
    // robot.arc(robot.cmToTacho(30), 90, -6.5, NONE);
    // lifo.setAccelParams(250, 20, 20);
    // lifo.distance(20, 5, NONE);
    // lifo.distance(45, 5, NONE);
    // lifo.lines(20, 1, COAST);
    // robot.setMode(REGULATED);
    // robot.arc(robot.cmToTacho(30), 90, -6, NONE);
    // lifo.distance(20, 5, NONE);
    // lifo.distance(45, 60, NONE);
    // lifo.lines(20, 1, COAST);


    // resetLifo();
    // lifo.initializeMotionMode(CONTROLLED);
    // lifo.setDoubleFollowMode("50", "SL");
    // lifo.setAccelParams(150, 20, 20);
    // lifo.setPIDparams(1.5, 3, 150, 1);
    // lifo.distance(20, 8, NONE);
    // lifo.setAccelParams(150, 20, 45);
    // lifo.distance(45, 25, NONE);
    // robot.setMode(CONTROLLED);
    // robot.setLinearAccelParams(200, 45, 45);
    // robot.straight(45, 20, NONE);
    // lifo.setAccelParams(150, 45, 20);
    // lifo.distance(45, 25, NONE);
    // lifo.setPIDparams(slowKP, slowKI, slowKD, PIDspeed);
    // lifo.setAccelParams(150, 20, 20);
    // lifo.lines(20, 1, NONE);


    // resetLifo();
    // lifo.setDoubleFollowMode("SL", "SR");
    // lifo.initializeMotionMode(CONTROLLED);
    // lifo.setPIDparams(1.5, 3, 150, 1);
    // lifo.setAccelParams(150, 20, 20);
    // lifo.distance(20, 8, NONE);
    // lifo.distance(45, 60, NONE);
    // setLifoSlow();
    // lifo.setAccelParams(150, 20, 20);
    // lifo.lines(20, 1, NONE);

    // robot.setMode(REGULATED);
    // robot.arc(robot.cmToTacho(35), 90, -6.5, NONE);

    // lifo.setPIDparams(1.5, 3, 150, 1);
    // lifo.distance(30, 8, NONE);
    // lifo.lines(20, 1, NONE);

    // robot.setMode(REGULATED);
    // robot.arc(robot.cmToTacho(35), 90, -6.5, NONE);

    // lifo.setDoubleFollowMode("SL", "50");
    // lifo.setAccelParams(150, 20, 20);
    // lifo.setPIDparams(1.5, 3, 150, 1);
    // lifo.distance(20, 8, NONE);
    // lifo.distance(45, 40, NONE);
    // setLifoSlow();
    // lifo.setAccelParams(150, 20, 20);
    // lifo.lines(20, 1, NONE);

    // robot.setMode(REGULATED);
    // robot.arc(robot.cmToTacho(35), 90, 6.5, NONE);

    // lifo.setDoubleFollowMode("SL", "SR");
    // lifo.setPIDparams(1.5, 3, 150, 1);
    // lifo.distance(30, 13, BRAKE);
    // robot.setMode(CONTROLLED);
    // robot.setAngularAccelParams(600, 200, 200);
    // robot.turn(300, 180);

    // lifo.setAccelParams(150, 20, 20);
    // lifo.setDoubleFollowMode("SL", "SR");
    // lifo.setPIDparams(1.5, 3, 150, 1);
    // lifo.distance(20, 8, NONE);
    // lifo.distance(45, 5, NONE);
    // setLifoSlow();
    // lifo.setAccelParams(150, 20, 20);
    // lifo.lines(20, 1, NONE);

    // robot.setMode(REGULATED);
    // robot.arc(robot.cmToTacho(35), 90, -6.5, NONE);

    // lifo.setDoubleFollowMode("50", "SR");
    // lifo.setAccelParams(150, 20, 20);
    // lifo.setPIDparams(1.5, 3, 150, 1);
    // lifo.distance(20, 8, NONE);
    // lifo.distance(45, 40, NONE);
    // setLifoSlow();
    // lifo.setAccelParams(150, 20, 20);
    // lifo.lines(20, 1, NONE);

    // robot.setMode(REGULATED);
    // robot.arc(robot.cmToTacho(35), 90, 6.5, NONE);

    // lifo.setDoubleFollowMode("SL", "SR");
    // lifo.setPIDparams(1.5, 3, 150, 1);
    // lifo.distance(30, 8, NONE);
    // lifo.lines(20, 1, NONE);

    // robot.setMode(REGULATED);
    // robot.arc(robot.cmToTacho(35), 90, 6.5, NONE);

    // lifo.initializeMotionMode(CONTROLLED);
    // lifo.setPIDparams(1.5, 3, 150, 1);
    // lifo.setAccelParams(150, 20, 20);
    // lifo.distance(20, 8, NONE);
    // lifo.distance(45, 50, NONE);
    // setLifoSlow();
    // lifo.setAccelParams(150, 20, 20);
    // lifo.lines(20, 1, NONE);


    // lifo.lines(20, 1, NONE);


    // control holder;
    // holder.setAbsoluteLimits(1150, 3000, 100);
    // holder.setPID(2, 1.2, 0.07, 0.002);
    // holder.setTargetTolerance(50, 5);
    // control controller;
    // controller.setAbsoluteLimits(1150, 3000, 1150);
    // controller.setPID(2, 1.2, 0.07, 0.002);
    // controller.setTargetTolerance(100, 15);
    // t.reset();
    // leftMotor.setMode(REGULATED);
    // rightMotor.setMode(REGULATED);
    // holder.stop();
    // leftMotor.resetTachoCount();
    // rightMotor.resetTachoCount();
    // controller.startPosition(516, 1000, 2000, 300, 300);
    // holder.startHold(0);
    // while(!controller.isDone())
    // {
    //     leftMotor.moveUnlimited(holder.update(t.secElapsed(), leftMotor.getTachoCount(), leftMotor.getCurrentSpeed()));
    //     rightMotor.moveUnlimited(controller.update(t.secElapsed(), rightMotor.getTachoCount(), rightMotor.getCurrentSpeed()));
    // }
    // holder.stop();
    // controller.stop();
    // leftMotor.stop(BRAKE);
    // rightMotor.stop(BRAKE);

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
    
    // ev3_motor_config(EV3_PORT_B, MEDIUM_MOTOR);
    // ev3_motor_config(EV3_PORT_C, MEDIUM_MOTOR);

    // // FILE *log = fopen("WRO2022/batteryTest.txt", "w");
    // FILE *log = bluetooth;

    // int setPower = 0;
    // int maxPower = 100;
    // bool isPowerAsc = true;
    // int leftReportedPower, rightReportedPower;
    // double leftActualSpeed, rightActualSpeed;
    // int voltage = 8000, current, power;
    // while(voltage > 7000)
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

    // robot.setMode(REGULATED);

    // double linearVelocity = 10;
    // double angularVelocity;
    // double k1 = 10, k2 = 100;
    // double r = 25, delta = 30, theta = -30;

    // double targetXpos = r * sin(delta);
    // double targetYpos = r * cos(delta);
    // double targetAngle = -delta + theta;

    // double angle = 0;
    // double x_pos = 0;
    // double y_pos = 0;
    // t.reset();
    // robot.resetPosition();
    // while(true)
    // {
    //     //Odometry
    //     angle += 0.02 * (-degToRad(robot.getAngularVelocity()));
    //     x_pos += 0.02 * (robot.getLinearVelocity() * cos(angle));
    //     y_pos += 0.02 * (robot.getLinearVelocity() * sin(angle));

    //     //Update Polar
    //     r = sqrt(pow(targetXpos - x_pos, 2) + pow(targetYpos - y_pos, 2));
    //     double rAngle = atan2(targetYpos - y_pos , targetXpos - x_pos) - MATH_PI / 2;
    //     delta = -(rAngle - angle);
    //     theta = -(rAngle - targetAngle);

    //     //Calculate Angluar Velocity
    //     angularVelocity = -(linearVelocity / r)*(k2 * (delta - atan(-k1*theta)) + (1 + k1 / (1 + pow(k1 * theta, 2)) * sin(delta)));
    //     angularVelocity = -radToDeg(angularVelocity);

    //     //Actuate
    //     robot.actuateKinematically(linearVelocity, angularVelocity);

    //     format(bt, "R: %  \tHeading: %  \n") %r %radToDeg(angle); 

    //     timer::secDelay(0.02);        
    // }
    // robot.stop(BRAKE);

    // format(bt, "Heading: %  degrees\t (x, y) = (%  ,%  )\n") %angle %x_pos %y_pos;


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

//GREATER CALIBRATION
// leftSensor.setNormalisation(false);
// rightSensor.setNormalisation(false);
// double minLeft[4] = {1000, 1000, 1000, 1000};
// double maxLeft[4] = {0, 0, 0, 0};
// double minRight[4] = {1000, 1000, 1000, 1000};
// double maxRight[4] = {0, 0, 0, 0};
// int minLeftRef = 100;
// int maxLeftRef = 0;
// int minRightRef = 100;
// int maxRightRef = 0;

// robot.setMode(CONTROLLED);
// robot.setLinearAccelParams(100, 20, 20);
// robot.straightUnlim(20, true);
// t.reset();

// while(t.secElapsed() < 5)
// {
//     colorspaceRGB leftRGB = leftSensor.getRGB();
//     colorspaceRGB rightRGB = rightSensor.getRGB();

//     minLeft[0] = min(minLeft[0], leftRGB.red);
//     minLeft[1] = min(minLeft[1], leftRGB.green);
//     minLeft[2] = min(minLeft[2], leftRGB.blue);
//     minLeft[3] = min(minLeft[3], leftRGB.white);

//     minRight[0] = min(minRight[0], rightRGB.red);
//     minRight[1] = min(minRight[1], rightRGB.green);
//     minRight[2] = min(minRight[2], rightRGB.blue);
//     minRight[3] = min(minRight[3], rightRGB.white);

//     maxLeft[0] = max(maxLeft[0], leftRGB.red);
//     maxLeft[1] = max(maxLeft[1], leftRGB.green);
//     maxLeft[2] = max(maxLeft[2], leftRGB.blue);
//     maxLeft[3] = max(maxLeft[3], leftRGB.white);

//     maxRight[0] = max(maxRight[0], rightRGB.red);
//     maxRight[1] = max(maxRight[1], rightRGB.green);
//     maxRight[2] = max(maxRight[2], rightRGB.blue);
//     maxRight[3] = max(maxRight[3], rightRGB.white);
// }

// robot.stop(BRAKE);

// btnEnter.waitForPress();

// t.reset();
// robot.straightUnlim(20, true);
// while(t.secElapsed() < 5)
// {
//     int left = leftSensor.getReflected();
//     int right = rightSensor.getReflected();

//     minLeftRef = min(minLeftRef, left);
//     minRightRef = min(minRightRef, right);

//     maxLeftRef = max(maxLeftRef, left);
//     maxRightRef = max(maxRightRef, right);
// }

// robot.stop(BRAKE);

// leftSensor.setRefCalParams(minLeftRef, maxLeftRef);
// rightSensor.setRefCalParams(minRightRef, maxRightRef);
// leftSensor.setRgbCalParams(minLeft, maxLeft);
// rightSensor.setRgbCalParams(minRight, maxRight);
