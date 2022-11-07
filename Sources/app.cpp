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
chassis robot(&leftMotor, &rightMotor, 6.24, 17.3, 0.06, 0.002, 0);
colorSensor leftSensor(SensorPort::S2, false, "WRO2022");
colorSensor rightSensor(SensorPort::S3, false, "WRO2022");
colorSensor leftScanner(SensorPort::S1, false, "WRO2022");
colorSensor rightScanner(SensorPort::S4, false, "WRO2022");
lineFollower lifo(700, &robot, &leftSensor, &rightSensor);
lineFollower lifoControlled(400, &robot, &leftSensor, &rightSensor);
lineFollower lifoUnregNormal(700, &robot, &leftSensor, &rightSensor);
lineFollower lifoUnregExtreme(400, &robot, &leftSensor, &rightSensor);
timer universalTimer;

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
lifoRobotPosition currentAlignment;

std::vector<int> graph[V];
std::map<std::pair<matPos, matPos>, routeFunc> routeMapping;

BrickButton btnEnter(BrickButtons::ENTER);

bool grabberUsed = false;
bool startPicking = false;
bool stopScanning = false;
int roomScanStage = 0;

colorSensor *scanner;
colors scannedValue;
colorSensor *lineDetector;
colors roomColor;

void startData()
{
    rooms.insert(pair<colors, room>(RED, room(RED)));
    rooms.insert(pair<colors, room>(GREEN, room(GREEN)));
    rooms.insert(pair<colors, room>(BLUE, room(BLUE)));
    rooms.insert(pair<colors, room>(YELLOW, room(YELLOW)));

    currentPos = S;
    currentDirection = NORTH;
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
    lifo.setPIDparams(0.5, 0.05, 5);  //UNREGULATED VALUES
    //lifo.setPIDparams(1, 1, 100, 1000);      //REGULATED VALUES

    leftSensor.setNormalisation(true);
    leftSensor.setFilteringRef(false);
    leftSensor.setFilteringRGB(false);
    leftSensor.setCutoffValue(27);
    rightSensor.setNormalisation(true);
    rightSensor.setFilteringRef(false);
    rightSensor.setFilteringRGB(false);
    rightSensor.setCutoffValue(27);

    leftScanner.setFilteringRef(false);
    leftScanner.setFilteringRGB(false);
    rightScanner.setFilteringRef(false);
    rightScanner.setFilteringRGB(false);
    leftScanner.setNormalisation(true);
    rightScanner.setNormalisation(true);
    
    grabber.setMode(REGULATED);
    ramp.setMode(REGULATED);
    //Initialize position of mechanisms (here or after start of mission, e.g. btn press)

    display.format("WAIT FOR SENSORS\n");
    btnEnter.waitForClick();
    // act_tsk(INIT_TASK);
    tslp_tsk(1);
}

void open_grabber_task(intptr_t unused)
{
    //OPENS GRABBER FULLY
    grabber.setMode(REGULATED);
    grabber.moveUnlimited(-1200, true);
    tslp_tsk(50);
    while(grabber.getCurrentSpeed() > -900)
    {
        grabber.moveUnlimited(-1200);
        tslp_tsk(1);
    }
    while(grabber.getCurrentSpeed() < -800)
    {
        grabber.moveUnlimited(-1200);
        tslp_tsk(1);
    }
    grabber.stop(BRAKE);
}

void init_task(intptr_t unused)
{
    //INITIALIZATION OF RAMP AND GRABBER
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
    ramp.moveUnlimited(-1000, true);
    tslp_tsk(150);
    while(abs(ramp.getCurrentSpeed()) > 400)
    {
        ramp.moveUnlimited(-1000);
        tslp_tsk(1);
    }
    ramp.stop(BRAKE);
}

void water_grabber_task(intptr_t unused)
{
    //PICKS ONE OBJECT AND IMMEDIATELY OPENS GRABBER TO BE READY FOR THE NEXT
    grabberUsed = true;
    grabber.moveDegrees(1200, 350, NONE);
    grabber.moveUnlimited(700, true);
    while(grabber.getCurrentSpeed() > 600)
    {
        grabber.moveUnlimited(700);
        tslp_tsk(1);
    }
    grabber.stop(COAST);
    grabber.setMode(REGULATED);
    grabber.moveUnlimited(-1200, true);
    tslp_tsk(50);
    while(grabber.getCurrentSpeed() > -900)
    {
        grabber.moveUnlimited(-1200);
        tslp_tsk(1);
    }
    while(grabber.getCurrentSpeed() < -800)
    {
        grabber.moveUnlimited(-1200);
        tslp_tsk(1);
    }
    grabber.stop(BRAKE);
    grabberUsed = false;
}

void pick_block_task(intptr_t unused)
{
    //RAISES GRABBER FULLY
    grabber.moveDegrees(1200, 350, NONE);
    grabber.moveUnlimited(700, true);
    while(grabber.getCurrentSpeed() > 600)
    {
        grabber.moveUnlimited(700);
        tslp_tsk(1);
    }
    grabber.stop(COAST);
}

void leave_ball_task(intptr_t unused)
{
    grabber.moveUnlimited(700, true);
    while(grabber.getCurrentSpeed() < 650)
    {
        grabber.moveUnlimited(700);
        tslp_tsk(1);
    }
    while(grabber.getCurrentSpeed() > 600)
    {
        grabber.moveUnlimited(700);
        tslp_tsk(1);
    }
    grabber.stop(COAST);
}

void empty_water_ramp_task(intptr_t unused)
{
    //LEAVES WATER ON TABLE FINAL STAGE
    emptyRampWaterStage2();
}

void basket_scan_task(intptr_t unused)
{
    //LOOKS FOR SIDE BASKETS
    colors leftBasket = BLACK;  //rightScanner used for left Basket
    colors rightBasket = BLACK; //leftScanner used for right Basket
    map<colors, int> appearancesLeft, appearancesRight;
    colors currentLeft, currentRight;
    bool notBlackLeft = false, notBlackRight = false;
    while(!stopScanning)
    {
        if((currentLeft = scanLaundryBasket(leftScanner)) != BLACK)
        {
            appearancesLeft[currentLeft]++;
            notBlackLeft = true;
        }
        if((currentRight = scanLaundryBasket(rightScanner)) != BLACK)
        {
            appearancesRight[currentRight]++;
            notBlackRight = true;
        }
        tslp_tsk(10);
    }
    if(notBlackLeft)
    {
        int maxCount = 0;
        for(auto x: appearancesLeft)
        {
            if(x.second > maxCount)
            {
                maxCount = x.second;
                rightBasket = x.first;    
            }
        }
    }
    if(notBlackRight)
    {
        int maxCount = 0;
        for(auto x: appearancesRight)
        {
            if(x.second > maxCount)
            {
                maxCount = x.second;
                leftBasket = x.first;    
            }
        }
    }

    laundryBaskets[BASKET_LEFT] = leftBasket;
    laundryBaskets[BASKET_RIGHT] = rightBasket;
}

void room_task_scan_task(intptr_t unused)
{
    colors current = NO_COLOR;
    map<colors, int> appearances;
    while(!stopScanning)
    {
        if((current = scanCodeBlock(*scanner)) != NO_COLOR)
            appearances[current]++;
        tslp_tsk(1);
    }
    current = analyzeFrequency(appearances, NO_COLOR);
    scannedValue = current;
}

void room_entrance_task(intptr_t unused)
{
    colors current = NO_COLOR;
    map<colors, int> appearances;

    roomScanStage = 1;  //until you find the entrance
    lineDetector->resetFiltering();
    while(!lineDetector->getLineDetected())
    {
        lineDetector->getRGB();
        tslp_tsk(5);
    }

    roomScanStage = 2;  //3 cm until before the laundry
    robot.resetPosition();
    while(robot.getPosition() < 3)
    {
        tslp_tsk(10);
    }

    roomScanStage = 3;  //until the bed (scanning)
    lineDetector->resetFiltering();
    while(!lineDetector->getLineDetected())
    {
        tslp_tsk(5);
        lineDetector->getRGB();
        if((current = scanLaundryBlock(*scanner)) != NO_COLOR)
            appearances[current]++;
    }

    current = analyzeFrequency(appearances, NO_COLOR);
    if(current != RED && current != YELLOW && current != NO_COLOR) current = BLACK;
    scannedValue = current;
    roomScanStage = 4;  //final distance based on room 
}

void human_scan_task(intptr_t unused)
{
    colors current = NO_COLOR;
    map<colors, int> appearances;
    while(!stopScanning)
    {
        if((current = leftScanner.getColor()) != NO_COLOR)
            appearances[current]++;
        if((current = rightScanner.getColor()) != NO_COLOR)
            appearances[current]++;
        tslp_tsk(1);
        scannedValue = analyzeFrequency(appearances, NO_COLOR);
    }
}

void end_task(intptr_t unused)
{
    //CLOSES EVERYTHING
    grabber.setMode(REGULATED);
    grabber.moveSeconds(500, 1, BRAKE);
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

    // grabber.stop();
    // // double min[4] = {1,2,1,2};
    // // double max[4] = {68,73,55,196};
    // // leftScanner.setRgbCalParams(min, max);
    // // leftScanner.setNormalisation(false);
    // // rightScanner.setNormalisation(false);
    // display.resetScreen();
    // while(true)
    // {
    //     // colorspaceRGB l = leftScanner.getRGB();
    //     // colorspaceRGB r = rightScanner.getRGB();
    //     // format(bt, "L: R: %  G: %  B: %  W: %  \nR: R: %  G: %  B: %  W: %  \n") %l.red %l.green %l.blue %l.white %r.red %r.green %r.blue %r.white;
    //     // colorspaceHSV l2 = leftScanner.getHSV();
    //     // colorspaceHSV r2 = rightScanner.getHSV();
    //     // format(bt, "L: H: %  S: %  V: %  \nR: H: %  S: %  V: %  \n\n") %l2.hue %l2.saturation %l2.value %r2.hue %r2.saturation %r2.value;
    //     display.format("L: %  \nR: %  \n\n\n") %static_cast<int>(leftScanner.getColor()) %static_cast<int>(rightScanner.getColor());
    //     tslp_tsk(10);
    // }


    // btnEnter.waitForClick();

    // grabber.stop();
    // // resetLifo();
    // // lifo.setPIDparams(KP*1.2, KI * 0.7, KD*1.5, 1);
    // // lifo.distance(robot.cmToTacho(30), 8, NONE);
    // // setLifoSlow();
    // // lifo.setAccelParams(150, 30, 30);
    // // lifo.distance(30, 6, NONE);
    // // lifo.lines(30, 1, NONE);
    // // lifo.distance(30, 1, NONE);
    // G_R(EAST);
    // rooms[BLUE].setTask(GREEN);
    // rooms[BLUE].executeAllActions();

    leftSensor.setFilteringRef(true, 0.01, 10);
    leftSensor.setFilteringRGB(true, 0.01, 30);

    rightSensor.setFilteringRef(true, 0.01, 10);
    rightSensor.setFilteringRGB(true, 0.01, 30);

    lifoControlled.initializeMotionMode(CONTROLLED);
    lifoControlled.setSensorMode(REFLECTED);
    lifoControlled.setAccelParams(100, 30, 30);
    lifoControlled.setPIDparams(2, 3, 70);  //HIGH CORRECTIONS
    lifoControlled.setPIDparams(2, 2, 60);  //SMOOTHER / FINISHER
    lifoControlled.setDoubleFollowMode("SL", "SR");

    lifoUnregNormal.initializeMotionMode(UNREGULATED);
    lifoUnregNormal.setSensorMode(REFLECTED);
    robot.setUnregulatedDPS(true);
    lifoUnregNormal.setDoubleFollowMode("SL", "SR");
    lifoUnregNormal.addPIDparams(30, 4, 1, 30);    //30 speed
    lifoUnregNormal.addPIDparams(40, 4, 3, 40);    //40 speed
    lifoUnregNormal.addPIDparams(50, 5, 4, 60);    //50 speed

    lifoUnregExtreme.initializeMotionMode(UNREGULATED);
    lifoUnregExtreme.setSensorMode(REFLECTED);
    robot.setUnregulatedDPS(true);
    lifoUnregExtreme.setDoubleFollowMode("SL", "SR");
    lifoUnregExtreme.addPIDparams(30, 4, 2, 50);    //30 speed
    lifoUnregExtreme.addPIDparams(40, 4, 4, 60);    //40 speed

//    // while(true)
//     // {
//     //     setLifo("SL", "SR");
//     //     // lifoControlled.setPIDparams(2, 1, 30);
//     //     // lifoControlled.lines(30, 1, NONE, 20, 9, true);
//     //     // lifoControlled.distance(30, 20, COAST);

//     //     robot.stop(BRAKE);
//     //     btnEnter.waitForClick();
//     // }

    while(true)
    {
        startProcedure();
        currentPos = S;
        fullRouteStandard(W);

        robot.setLinearAccelParams(100, 40, 30);
        robot.straight(40, 5, NONE);
        robot.setLinearAccelParams(100, 30, 0);
        robot.straight(30, 3, COAST);

        robot.setLinearAccelParams(100, 0, 0);
        robot.straight(30, -2, COAST);
        robot.arc(45, -125, 4, COAST);
        robot.arc(45, -35, -8.5, COAST);
        robot.straight(45, -9, COAST);

        colors waters[3];
        int idx=0;

        rightScanner.setColorDataSat(rightScanner.getColorDataSat() / 2.0);

        setLifo("SR", "50");
        stopScanning = false;
        act_tsk(HUMAN_SCAN_TASK);
        tslp_tsk(1);
        lifoUnregExtreme.distance(30, 5, NONE);
        stopScanning = true;
        waters[idx++] = scannedValue; 
        stopScanning = false;
        ter_tsk(HUMAN_SCAN_TASK);
        tslp_tsk(1);
        act_tsk(HUMAN_SCAN_TASK);
        tslp_tsk(1);
        lifoUnregNormal.lines(30, 1, NONE, 3, 2, true);
        lifoUnregNormal.distance(30, 2.5, NONE);
        stopScanning = true;
        waters[idx++] = scannedValue; 
        stopScanning = false;
        ter_tsk(HUMAN_SCAN_TASK);
        tslp_tsk(1);
        act_tsk(HUMAN_SCAN_TASK);
        tslp_tsk(1);
        lifoUnregNormal.distance(30, 10, COAST);
        stopScanning = true;
        waters[idx++] = scannedValue; 
        stopScanning = false;
        ter_tsk(HUMAN_SCAN_TASK);
        tslp_tsk(1);
        act_tsk(INIT_TASK);
        tslp_tsk(1);
        robot.arc(45, -90, -8.5, COAST);

        setLifo("SL", "SR");  
        lifoUnregExtreme.lines(30, 1, NONE, 5);

        display.resetScreen();
        for(auto x : waters)
        {
            display.format("%  \n")%static_cast<int>(x);
        }

        pickWater();

        robot.stop(BRAKE);
        btnEnter.waitForClick();
    }

    // while(true)
    // {
    //     currentPos = M;
    //     currentDirection = NORTH;
    //     currentAlignment = CENTERED;

    //     fullRouteStandard(TL);

    //     //TL -> TLLH
    //     robot.setMode(CONTROLLED);
    //     robot.setLinearAccelParams(100, 0, -40);
    //     robot.arc(40, -48, 8.5, NONE);
    //     act_tsk(OPEN_GRABBER_TASK);
    //     tslp_tsk(1);
    //     robot.setLinearAccelParams(100, -40, 0);
    //     robot.arc(40, -48, 8.5, COAST);

    //     robot.setLinearAccelParams(100, 0, 0);
    //     robot.straight(45, 15, COAST);

    //     grabber.moveUnlimited(600, true);
    //     while(grabber.getCurrentSpeed() < 570)
    //     {
    //         grabber.moveUnlimited(600);
    //         tslp_tsk(1);
    //     }
    //     while(grabber.getCurrentSpeed() > 550)
    //     {
    //         grabber.moveUnlimited(600);
    //         tslp_tsk(1);
    //     }
    //     grabber.stop(BRAKE);

    //     robot.straight(45, -12,  COAST);
    //     robot.arc(45, -90, 0, COAST);
    //     robot.setLinearAccelParams(100, 0, 30);
    //     robot.straight(30, 3, NONE);

    //     currentDirection = SOUTH;
    //     fullRouteStandard(M);

    //     finishProcedure();


    //     // fullRouteStandard(TLH);

    //     // stopScanning = false;
    //     // scanner = &leftScanner;
    //     // act_tsk(HUMAN_SCAN_TASK);
    //     // tslp_tsk(1);
    //     // robot.arc(45, 20, 20, COAST);
    //     // robot.arc(45, -20, 20, COAST);
    //     // stopScanning = true;

    //     // fullRouteStandard(M);        
    //     // display.resetScreen();
    //     // display.format("%  \n")%static_cast<int>(scannedValue);


    //     // fullRouteStandard(CL1);
    //     // fullRouteStandard(TLH);
    //     // fullRouteStandard(CL1);


    //     // fullRouteStandard(BR);
    //     // fullRouteStandard(CR2);
    //     // fullRouteStandard(BR);
    //     // fullRouteStandard(TR);

    //     // fullRouteStandard(RR2);
    //     // fullRouteStandard(GR1);
    //     // fullRouteStandard(GR2);
    //     // fullRouteStandard(RR1);
    //     // fullRouteStandard(BR);
    //     // fullRouteStandard(CR2);
    //     // fullRouteStandard(TR);
    //     // fullRouteStandard(BR);
    //     // fullRouteStandard(TR);
    //     // fullRouteStandard(CR3);
    //     // fullRouteStandard(CR1);


    //     robot.stop(BRAKE);
    //     btnEnter.waitForClick();
    // }

//     //START, WATER
//     while(true)
//     {
//         // act_tsk(INIT_TASK);
//         // tslp_tsk(1);

//         startProcedure();

//         S_W(NORTH);

//         pickWaterLast();

//         // lifoUnregExtreme.distance(30, 10, NONE);
//         // lifoUnregNormal.lines(40, 1, NONE, 10);

//         // W_IR(EAST);

//         // IR_G(EAST);

//         // rooms[GREEN].executeAllActions();

//         // G_R(NORTH);

//         // rooms[RED].executeAllActions();

//         // R_IR(SOUTH);

//         // act_tsk(CLOSE_RAMP_TASK);
//         // tslp_tsk(1);
//         // while(!rampQueue.empty()) rampQueue.pop();
//         // rampQueue.push(LAUNDRY_BLACK);
//         // rampQueue.push(LAUNDRY_RED);
//         // rampQueue.push(LAUNDRY_YELLOW);
//         // rooms[RED].setTask(GREEN);
//         // rooms[GREEN].setTask(WHITE);

//         // IR_IL(WEST);

//         // IL_B(WEST);

//         // rooms[BLUE].executeAllActions();

//         // B_Y(NORTH);

//         // rooms[YELLOW].executeAllActions();

//         // Y_IL(SOUTH);
//         // IL_L(EAST);

//         // scanLaundryBaskets();

//         // leaveLaundry();

//         // L_S(NORTH);

//         // finishProcedure();

//         robot.stop(BRAKE);
//         btnEnter.waitForClick();
//     }

//     //GREEN, RED ROOM
//     while(true)
//     {
//         act_tsk(CLOSE_RAMP_TASK);
//         tslp_tsk(1);
//         rampQueue.push(BOTTLE);
//         rampQueue.push(BOTTLE);

//         setLifo("SL", "70");
//         lifoUnregExtreme.distance(30, 5, NONE);
//         lifoUnregExtreme.distance(40, 5, NONE);
//         lifoUnregNormal.lines(40, 1, NONE, 5, 8.5, true);
//         lifoUnregNormal.distance(30, 5, NONE);
//         lifoControlled.lines(30, 1, NONE, 5);

//         robot.setLinearAccelParams(100, 30, 0);
//         robot.straight(30, 8, COAST);

//         robot.setLinearAccelParams(100, 0, -25);
//         robot.arc(50, -85, -5, NONE);
//         robot.setLinearAccelParams(100, -25, -25);
//         robot.arcUnlim(25, -5, BACKWARD, true);
//         while(leftSensor.getReflected() > 80 && abs(robot.getAngle()) < 10)
//             robot.arcUnlim(25, -5, BACKWARD, false);
//         robot.stop(COAST);

//         setLifo("SL", "70");
//         lifoUnregNormal.distance(30, 12, NONE);

//         rooms[GREEN].setTask(GREEN);
//         rooms[GREEN].executeAllActions();

//         setLifo("70", "SR");
//         lifoUnregExtreme.distance(30, 5, NONE);
//         lifoUnregExtreme.distance(40, 5, NONE);
//         lifoUnregNormal.lines(40, 1, NONE, 5, 8.5, true);
//         lifoUnregNormal.distance(30, 12, NONE);

//         rooms[RED].setTask(GREEN);
//         rooms[RED].executeAllActions();

//         robot.stop(BRAKE);
//         btnEnter.waitForClick();
//     }

//     //BLUE, YELLOW ROOM
//     while(true)
//     {
//         act_tsk(CLOSE_RAMP_TASK);
//         tslp_tsk(1);
//         rampQueue.push(BOTTLE);
//         rampQueue.push(BOTTLE);

//         setLifo("70", "SR");
//         lifoUnregExtreme.distance(30, 5, NONE);
//         lifoUnregExtreme.distance(40, 5, NONE);
//         lifoUnregNormal.lines(40, 1, NONE, 5, 8.5, true);
//         lifoUnregNormal.distance(30, 5, NONE);
//         lifoControlled.lines(30, 1, NONE, 5);

//         robot.setLinearAccelParams(100, 30, 0);
//         robot.straight(30, 8, COAST);

//         robot.setLinearAccelParams(100, 0, -25);
//         robot.arc(50, -85, 5, NONE);
//         robot.setLinearAccelParams(100, -25, -25);
//         robot.arcUnlim(25, 5, BACKWARD, true);
//         while(rightSensor.getReflected() > 80 && abs(robot.getAngle()) < 10)
//             robot.arcUnlim(25, 5, BACKWARD, false);
//         robot.stop(COAST);

//         setLifo("70", "SR");
//         lifoUnregNormal.distance(30, 12, NONE);
//         // lifoControlled.distance(30, 10, NONE);

//         rooms[BLUE].setTask(WHITE);
//         rooms[BLUE].executeAllActions();

//         setLifo("SL", "70");
//         lifoUnregExtreme.distance(30, 5, NONE);
//         lifoUnregExtreme.distance(40, 5, NONE);
//         lifoUnregNormal.lines(40, 1, NONE, 5, 8.5, true);
//         lifoUnregNormal.distance(30, 12, NONE);

//         // robot.stop(BRAKE);
//         // btnEnter.waitForClick();


//         // setLifo("SL", "70");
//         // lifoUnregExtreme.distance(30, 5, NONE);
//         // lifoUnregExtreme.distance(40, 5, NONE);
//         // lifoUnregNormal.lines(40, 1, NONE, 5, 8.5, true);
//         // lifoUnregNormal.distance(30, 5, NONE);
//         // lifoControlled.lines(30, 1, NONE, 5);

//         // robot.setLinearAccelParams(100, 30, 0);
//         // robot.straight(30, 8, COAST);

//         // robot.setLinearAccelParams(100, 0, -25);
//         // robot.arc(50, -85, -5, NONE);
//         // robot.setLinearAccelParams(100, -25, -25);
//         // robot.arcUnlim(25, -5, BACKWARD, true);
//         // while(leftSensor.getReflected() > 80 && abs(robot.getAngle()) < 10)
//         //     robot.arcUnlim(25, -5, BACKWARD, false);
//         // robot.stop(COAST);

//         // setLifo("SL", "70");
//         // lifoUnregNormal.distance(30, 12, NONE);
//         // // lifoControlled.distance(30, 10, NONE);

//         rooms[YELLOW].setTask(WHITE);
//         rooms[YELLOW].executeAllActions();

//         robot.stop(BRAKE);
//         btnEnter.waitForClick();
//     }

//     //RANDOM ROUTES (will be used for auxiliary network)
//     while(true)
//     {
//         currentDirection = NORTH;    
//         currentAlignment = CENTERED;

//         setLifo("SL", "SR");
//         lifoUnregExtreme.distance(30, 5, NONE);
//         lifoUnregNormal.distance(40, 50, NONE);
//         lifoUnregNormal.distance(30, 5, NONE);
//         lifoControlled.lines(30, 1, NONE, 5);

//         standardTurn(currentDirection, WEST, CENTERED);
//         lifoUnregExtreme.distance(30, 5, NONE);
//         lifoControlled.lines(30, 1, NONE, 5);

//         standardTurn(currentDirection, SOUTH, CENTERED);
//         setLifo("SL", "50");
//         lifoUnregExtreme.distance(30, 5, NONE);
//         lifoUnregNormal.lines(40, 3, NONE, 5, 2, true);
//         lifoUnregNormal.distance(40, 10, NONE);
//         lifoUnregNormal.distance(30, 5, NONE);
//         lifoControlled.lines(30, 1, NONE, 5);

//         standardTurn(currentDirection, WEST, CENTERED);
//         setLifo("SL", "SR");
//         lifoUnregExtreme.distance(30, 5, NONE);
//         lifoUnregNormal.distance(40, 10, NONE);
//         lifoUnregNormal.distance(30, 5, NONE);
//         lifoControlled.lines(30, 1, NONE, 5);
        
//         standardTurn(currentDirection, EAST, RIGHT_OF_LINE);
//         setLifo("50", "SL");
//         lifoUnregExtreme.distance(30, 10, NONE);
//         lifoUnregNormal.distance(40, 20, NONE);
//         lifoUnregNormal.distance(30, 5, NONE);
//         lifoControlled.lines(30, 1, NONE, 5);

//         robot.setLinearAccelParams(100, 30, 0);
//         robot.straight(30, 5, COAST);
//         standardTurn(currentDirection, WEST, RIGHT_OF_LINE);
//         lifoControlled.lines(30, 1, NONE, 1);

//         standardTurn(currentDirection, NORTH, RIGHT_OF_LINE);
//         setLifo("50", "SL");
//         lifoUnregExtreme.distance(30, 5, NONE);
//         lifoUnregExtreme.distance(40, 5, NONE);
//         lifoUnregNormal.distance(40, 60, NONE);
//         lifoControlled.lines(30, 1, NONE, 5);

//         standardTurn(currentDirection, SOUTH, CENTERED);
//         setLifo("SL", "SR");
//         lifoUnregExtreme.distance(30, 5, NONE);
//         lifoUnregNormal.lines(40, 1, NONE, 5, 2, true);
//         lifoUnregNormal.distance(40, 5, NONE);
//         lifoUnregNormal.distance(30, 5, NONE);
//         lifoControlled.lines(30, 1, NONE, 5);

//         standardTurn(currentDirection, WEST, CENTERED);
//         lifoControlled.setSensorMode(WHITE_RGB);
//         lifoUnregExtreme.distance(30, 5, NONE);
//         // lifoUnregNormal.distance(40, 5, NONE);
//         lifoUnregNormal.distance(30, 5, NONE);
//         lifoControlled.lines(30, 1, NONE, 5);
//         lifoControlled.setSensorMode(REFLECTED);

//         standardTurn(currentDirection, EAST, LEFT_OF_LINE);
//         setLifo("SR", "50");
//         lifoUnregExtreme.distance(30, 5, NONE);
//         // lifoUnregNormal.distance(40, 5, NONE);
//         lifoUnregNormal.distance(30, 5, NONE);
//         lifoControlled.lines(30, 1, NONE, 5);

//         standardTurn(currentDirection, SOUTH, RIGHT_OF_LINE);
//         setLifo("50", "SL");
//         lifoUnregExtreme.distance(30, 5, NONE);
//         lifoControlled.lines(30, 1, NONE, 5);
        
//         standardTurn(currentDirection, WEST, LEFT_OF_LINE);
//         lifoControlled.setSensorMode(WHITE_RGB);
//         setLifo("SR", "50");
//         lifoUnregExtreme.distance(30, 5, NONE);
//         // lifoUnregNormal.distance(40, 5, NONE);
//         lifoUnregNormal.distance(30, 5, NONE);
//         lifoControlled.lines(30, 1, NONE, 5);
//         lifoControlled.setSensorMode(REFLECTED);

//         standardTurn(currentDirection, EAST, CENTERED);
//         setLifo("SL", "SR");
//         lifoUnregExtreme.distance(30, 5, NONE);
//         // lifoUnregNormal.distance(40, 5, NONE);
//         lifoUnregNormal.distance(30, 5, NONE);
//         lifoControlled.lines(30, 1, NONE, 5);

//         standardTurn(currentDirection, NORTH, CENTERED);
//         setLifo("50", "SR");
//         lifoUnregExtreme.distance(30, 5, NONE);
//         lifoUnregNormal.lines(40, 2, NONE, 5, 2, true);
//         lifoUnregNormal.distance(40, 10, NONE);
//         lifoUnregNormal.distance(30, 5, NONE);
//         lifoControlled.lines(30, 1, NONE, 5);


//         robot.stop(BRAKE);
//         btnEnter.waitForClick();
//     }


    startProcedure();
    
    fullRouteStandard(W);
    pickWater();
    
    fullRouteStandard(G);
    rooms[GREEN].executeAllActions();
    fullRouteStandard(R);
    rooms[RED].executeAllActions();
    fullRouteStandard(B);
    rooms[BLUE].executeAllActions();
    fullRouteStandard(Y);
    rooms[YELLOW].executeAllActions();
    fullRouteStandard(L);

    scanLaundryBaskets();
    leaveLaundry();

    fullRouteStandard(S);
    finishProcedure();

    robot.stop(BRAKE);

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

//SIMPLE CALIBRATION RGB

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

//SIMPLE CALIBRATION REF
// leftSensor.setNormalisation(false);
// rightSensor.setNormalisation(false);
// double minLeft = 1000;
// double minRight = 1000;
// double maxLeft = 0;
// double maxRight = 0;
// t.secDelay(1);
// display.resetScreen();
// display.format("LEFT -> BLACK");
// btnEnter.waitForPress();
// minLeft = leftSensor.getReflected();
// t.secDelay(1);
// display.resetScreen();
// display.format("LEFT -> WHITE");
// btnEnter.waitForPress();
// maxLeft = leftSensor.getReflected();
// t.secDelay(1);
// display.resetScreen();
// display.format("RIGHT -> BLACK");
// btnEnter.waitForPress();
// minRight = rightSensor.getReflected();
// t.secDelay(1);
// display.resetScreen();
// display.format("RIGHT -> WHITE");
// btnEnter.waitForPress();
// maxRight = rightSensor.getReflected();
// leftSensor.setRefCalParams(minLeft, maxLeft);
// rightSensor.setRefCalParams(minRight, maxRight);
// btnEnter.waitForClick();

//GREATER LINE SENSOR CALIBRATION
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
    //
    // robot.setMode(CONTROLLED);
    // robot.setLinearAccelParams(100, 20, 20);
    // robot.straightUnlim(20, true);
    // t.reset();
    //
    // while(t.secElapsed() < 2)
    // {
    //     rgb_raw_t leftRGB; ev3_color_sensor_get_rgb_raw(EV3_PORT_2, &leftRGB);
    //     rgb_raw_t rightRGB; ev3_color_sensor_get_rgb_raw(EV3_PORT_3, &rightRGB);
    //
    //     minLeft[0] = min(minLeft[0], (double)leftRGB.r);
    //     minLeft[1] = min(minLeft[1], (double)leftRGB.g);
    //     minLeft[2] = min(minLeft[2], (double)leftRGB.b);
    //     minLeft[3] = min(minLeft[3], (double)leftRGB.r+leftRGB.g+leftRGB.b);
    //
    //     minRight[0] = min(minRight[0], (double)rightRGB.r);
    //     minRight[1] = min(minRight[1], (double)rightRGB.g);
    //     minRight[2] = min(minRight[2], (double)rightRGB.b);
    //     minRight[3] = min(minRight[3], (double)rightRGB.r+rightRGB.g+rightRGB.b);
    //
    //     maxLeft[0] = max(maxLeft[0], (double)leftRGB.r);
    //     maxLeft[1] = max(maxLeft[1], (double)leftRGB.g);
    //     maxLeft[2] = max(maxLeft[2], (double)leftRGB.b);
    //     maxLeft[3] = max(maxLeft[3], (double)leftRGB.r+leftRGB.g+leftRGB.b);
    //
    //     maxRight[0] = max(maxRight[0], (double)rightRGB.r);
    //     maxRight[1] = max(maxRight[1], (double)rightRGB.g);
    //     maxRight[2] = max(maxRight[2], (double)rightRGB.b);
    //     maxRight[3] = max(maxRight[3], (double)rightRGB.r+rightRGB.g+rightRGB.b);
    // }
    //
    // robot.stop(BRAKE);
    //
    // btnEnter.waitForPress();
    //
    // t.reset();
    // robot.straightUnlim(20, true);
    // while(t.secElapsed() < 2)
    // {
    //     int left = leftSensor.getReflected();
    //     int right = rightSensor.getReflected();
    //
    //     minLeftRef = min(minLeftRef, left);
    //     minRightRef = min(minRightRef, right);
    //
    //     maxLeftRef = max(maxLeftRef, left);
    //     maxRightRef = max(maxRightRef, right);
    // }
    //
    // robot.stop(BRAKE);
    //
    // leftSensor.setRefCalParams(minLeftRef, maxLeftRef);
    // rightSensor.setRefCalParams(minRightRef, maxRightRef);
    // leftSensor.setRgbCalParams(minLeft, maxLeft);
    // rightSensor.setRgbCalParams(minRight, maxRight);

// //SCANNER CALIBRATION 
    // //Min measurements done with BLACK block 4 STUDS away
    // //Max measurements done with WHITE block 0.5 STUDS away
    // double minL[4] = {0,0,0,0};
    // double maxL[4] = {0,0,0,0};
    // double minR[4] = {0,0,0,0};
    // double maxR[4] = {0,0,0,0};
    //
    // leftScanner.setNormalisation(false);
    // rightScanner.setNormalisation(false);
    // display.resetScreen();
    // BrickButton right(BrickButtons::RIGHT);
    // t.secDelay(1);
    // colorspaceRGB sum = {0, 0, 0, 0};
    // int times = 0;
    // display.format("Calibrating Left Scanner Min");
    // while(!btnEnter.isPressed())
    // {
    //     while(!right.isPressed() && !btnEnter.isPressed());
    //     rgb_raw_t l; ev3_color_sensor_get_rgb_raw(EV3_PORT_1, &l);
    //     format(bt, "LL: R: %  G: %  B: %  W:   \n") %l.r %l.g %l.b;
    //     sum.red += l.r;
    //     sum.green += l.g;
    //     sum.blue += l.b;
    //     times++;
    //     while(right.isPressed());
    // }
    // minL[0] = sum.red / (double)times;
    // minL[1] = sum.green / (double)times;
    // minL[2] = sum.blue / (double)times;
    // minL[3] = (sum.red+sum.green+sum.blue) / (double)times;
    // t.secDelay(1);
    // sum.red = sum.green = sum.blue = times = 0;
    // display.resetScreen();
    // display.format("Calibrating Left Scanner Max");
    // while(!btnEnter.isPressed())
    // {
    //     while(!right.isPressed() && !btnEnter.isPressed());
    //     rgb_raw_t l; ev3_color_sensor_get_rgb_raw(EV3_PORT_1, &l);
    //     format(bt, "LL: R: %  G: %  B: %  W:   \n") %l.r %l.g %l.b;
    //     sum.red += l.r;
    //     sum.green += l.g;
    //     sum.blue += l.b;
    //     times++;
    //     while(right.isPressed());
    // }
    // maxL[0] = sum.red / (double)times;
    // maxL[1] = sum.green / (double)times;
    // maxL[2] = sum.blue / (double)times;
    // maxL[3] = (sum.red+sum.green+sum.blue) / (double)times;
    // leftScanner.setRgbCalParams(minL, maxL);
    // leftScanner.setNormalisation(true);
    // display.resetScreen();
    // display.format("Calibration Left Scanner DONE");
    // format(bt, "LL: R: %  G: %  B: %  W:   \nRR: R: %  G: %  B: %  W:   \n") %minL[0] %minL[1] %minL[2] %maxL[0] %maxL[1] %maxL[2];
    //
    //
    // t.secDelay(1);
    // sum.red = sum.green = sum.blue = times = 0;
    // display.format("Calibrating Right Scanner Min");
    // while(!btnEnter.isPressed())
    // {
    //     while(!right.isPressed() && !btnEnter.isPressed());
    //     rgb_raw_t r; ev3_color_sensor_get_rgb_raw(EV3_PORT_4, &r);
    //     format(bt, "LL: R: %  G: %  B: %  W:   \n") %r.r %r.g %r.b;
    //     sum.red += r.r;
    //     sum.green += r.g;
    //     sum.blue += r.b;
    //     times++;
    //     while(right.isPressed());
    // }
    // minR[0] = sum.red / (double)times;
    // minR[1] = sum.green / (double)times;
    // minR[2] = sum.blue / (double)times;
    // minR[3] = (sum.red+sum.green+sum.blue) / (double)times;
    // t.secDelay(1);
    // sum.red = sum.green = sum.blue = times = 0;
    // display.resetScreen();
    // display.format("Calibrating Right Scanner Max");
    // while(!btnEnter.isPressed())
    // {
    //     while(!right.isPressed() && !btnEnter.isPressed());
    //     rgb_raw_t r; ev3_color_sensor_get_rgb_raw(EV3_PORT_4, &r);
    //     format(bt, "LL: R: %  G: %  B: %  W:   \n") %r.r %r.g %r.b;
    //     sum.red += r.r;
    //     sum.green += r.g;
    //     sum.blue += r.b;
    //     times++;
    //     while(right.isPressed());
    // }
    // maxR[0] = sum.red / (double)times;
    // maxR[1] = sum.green / (double)times;
    // maxR[2] = sum.blue / (double)times;
    // maxR[3] = (sum.red+sum.green+sum.blue) / (double)times;
    // rightScanner.setRgbCalParams(minR, maxR);
    // rightScanner.setNormalisation(true);
    // display.resetScreen();
    // display.format("Calibration Right Scanner DONE");
    // format(bt, "minL: R: %  G: %  B: %  W:   \nmaxL: R: %  G: %  B: %  W:   \n") %minL[0] %minL[1] %minL[2] %maxL[0] %maxL[1] %maxL[2];
    // format(bt, "minR: R: %  G: %  B: %  W:   \nmaxR: R: %  G: %  B: %  W:   \n") %minR[0] %minR[1] %minR[2] %maxR[0] %maxR[1] %maxR[2];
    //
    // color_hue hues[5] = {{RED, 10, 20}, {RED, 350, 20}, {GREEN, 140, 80}, 
    //                     {BLUE, 225, 50}, {YELLOW, 35, 30}};
    //
    // leftScanner.setColorCalParams(hues, 5, 70, 15);
    // rightScanner.setColorCalParams(hues, 5, 70, 15);

//Motor benchmark
    // ev3_motor_config(EV3_PORT_B, MEDIUM_MOTOR);
    // ev3_motor_config(EV3_PORT_C, MEDIUM_MOTOR);
    //
    // // FILE *log = fopen("WRO2022/batteryTest.txt", "w");
    // FILE *log = bluetooth;
    //
    // int setPower = 50;
    // int maxPower = 85;
    // int minPower = 40;
    // bool isPowerAsc = true;
    // int leftReportedPower, rightReportedPower;
    // double leftActualSpeed, rightActualSpeed;
    // int voltage = 8000, current, power;
    // while(voltage > 7000)
    // {
    //     ev3_motor_set_power(EV3_PORT_B, -setPower);
    //     ev3_motor_set_power(EV3_PORT_C, setPower);
    //     ev3_motor_reset_counts(EV3_PORT_B);
    //     ev3_motor_reset_counts(EV3_PORT_C);
    //     t.reset();
    //     t.secDelay(0.01);
    //     leftReportedPower = ev3_motor_get_power(EV3_PORT_B);
    //     rightReportedPower = ev3_motor_get_power(EV3_PORT_C);
    //     leftActualSpeed = ev3_motor_get_counts(EV3_PORT_B) / t.secElapsed();
    //     rightActualSpeed = ev3_motor_get_counts(EV3_PORT_C) / t.secElapsed();
    //     voltage = ev3_battery_voltage_mV();
    //     current = ev3_battery_current_mA();
    //     power = voltage * current;
    //
    //
    //     if(setPower == 0)
    //     {
    //         ev3_motor_stop(EV3_PORT_B, false);
    //         ev3_motor_stop(EV3_PORT_C, false);
    //
    //         t.secDelay(5);
    //
    //         voltage = ev3_battery_voltage_mV();
    //         current = ev3_battery_current_mA();
    //         power = voltage * current;
    //         fprintf(log, "%d\t%d\t%d\t%lf\t%lf\t%d\t%d\t%d\n", 0, 0, 0, 0.0, 0.0, voltage, current, power);
    //     }
    //     else
    //     {
    //         fprintf(log, "%d\t%d\t%d\t%lf\t%lf\t%d\t%d\t%d\n", setPower, leftReportedPower, rightReportedPower, leftActualSpeed, rightActualSpeed, voltage, current, power);
    //     }
    //  
    //     if(abs(setPower) < minPower)
    //         setPower = -sign(setPower) * minPower;
    //
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

/*//Calibration Debug
vector<char*> ports = {"S1", "S2", "S3", "S4"};
    for(auto port : ports)
    {
        tslp_tsk(1);
        double offsetRef, scaleRef;
        bool normalisedRef = true;
        char filename[32];
        sprintf(filename, "%s/%srefCal.txt", "WRO2022", port);
        FILE *calData = fopen(filename, "r+");
        if(calData != nullptr)
        {
            fscanf(calData, "%lf", &offsetRef);
            fscanf(calData, "%lf", &scaleRef);
        }
        else
            normalisedRef = false;
        fclose(calData);
        
        
        if(normalisedRef)
        {
            DEBUGPRINT("%s: REF:\tOffset:%lf\tScale:%lf\n", port, offsetRef, scaleRef);
        }
        tslp_tsk(1);
        double offsetRGB[4] = {0, 0, 0, 0}, scaleRGB[4] = {0, 0, 0, 0};
        bool normalisedRGB = true;
        sprintf(filename, "%s/%srgbCal.txt", "WRO2022", port);
        calData = fopen(filename, "r+");
        if(calData != nullptr)
        {
            for (int i = 0; i < 4; i++)
            {
                fscanf(calData, "%lf", &offsetRGB[i]);
                fscanf(calData, "%lf", &scaleRGB[i]);
            }
        }
        else
            normalisedRGB = false;
        
        fclose(calData);
        if(normalisedRGB)
        {
            DEBUGPRINT("%s: RGB:\nR:\tOffset:%lf\tScale:%lf\nG:\tOffset:%lf\tScale:%lf\nB:\tOffset:%lf\tScale:%lf\nW:\tOffset:%lf\tScale:%lf\n", port, offsetRGB[0], scaleRGB[0], offsetRGB[1], scaleRGB[1], offsetRGB[2], scaleRGB[2], offsetRGB[3], scaleRGB[3]);
        }
        tslp_tsk(1);
        sprintf(filename, "%s/%scolorCal.txt", "WRO2022", port);
        calData = fopen(filename, "r+");
        int colorsAmount = 5;
        colorCalibration colorData;
        colorData.hues.clear();
        if(calData != nullptr)
        {
            fscanf(calData, "%lf", &colorData.minColorSaturation);
            fscanf(calData, "%lf", &colorData.greyscaleIntersection);
            for (int i = 0; i < colorsAmount; i++)
            {
                color_hue data;
                fscanf(calData, "%d", &data.color);
                fscanf(calData, "%lf", &data.hue);
                fscanf(calData, "%lf", &data.zoneSize);
                colorData.hues.push_back(data);
            }
        }
        if(calData != nullptr)
        {
            DEBUGPRINT("%s: Color:\n", port);
            DEBUGPRINT("MinColorSaturation: %lf\n", colorData.minColorSaturation);
            DEBUGPRINT("GreyscaleIntersection: %lf\n", colorData.greyscaleIntersection);
            for (int i = 0; i < colorsAmount; i++)
            {
                DEBUGPRINT("Color: %d\t", colorData.hues[i].color);
                DEBUGPRINT("Mid Hue: %lf\t", colorData.hues[i].hue);
                DEBUGPRINT("Hue Range Size: %lf\n", colorData.hues[i].zoneSize);
            }
        }
        fclose(calData);
    }
    */
