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
bool stopScanning = false;
int scanStage = 0;

void open_grabber_task(intptr_t unused)
{
    //INITIALIZATION OF GRABBER
    grabber.setMode(REGULATED);
    grabber.moveUnlimited(-1000, true);
    tslp_tsk(50);
    while(grabber.getCurrentSpeed() > -450)
    {
        grabber.moveUnlimited(-1000);
        tslp_tsk(1);
    }
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
    // ramp.setMode(REGULATED);
    // ramp.moveUnlimited(-1000, true);
    // tslp_tsk(50);
    // while(abs(ramp.getCurrentSpeed()) > 400)
    // {
    //     ramp.moveUnlimited(-1000);
    //     tslp_tsk(1);
    // }
    // ramp.stop(BRAKE);

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
    grabber.stop(BRAKE_COAST);
}

void empty_water_ramp_task(intptr_t unused)
{
    //LEAVES WATER ON TABLE FINAL STAGE
    emptyRampWaterStage2();
}

void basket_scan_task(intptr_t unused)
{
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

    // colors leftBasket = BLACK;  //rightScanner used for left Basket
    // colors middleBasket = BLACK; //leftScanner used for right Basket
    // map<colors, int> appearancesLeft, appearancesMiddle;
    // colors currentLeft, currentMiddle;
    // bool notBlackLeft = false, notBlackMiddle = false;
    // while(scanStage == 1)
    // {
    //     if((currentMiddle = scanLaundryBasket(leftScanner)) != BLACK)
    //     {
    //         appearancesMiddle[currentMiddle]++;
    //         notBlackMiddle = true;
    //     }
    //     tslp_tsk(10);
    // }
    // while(scanStage == 2)
    // {
    //     if((currentLeft = scanLaundryBasket(leftScanner)) != BLACK)
    //     {
    //         appearancesLeft[currentLeft]++;
    //         notBlackLeft = true;
    //     }
    //     tslp_tsk(10);
    // }

    // if(notBlackMiddle)
    // {
    //     int maxCount = 0;
    //     for(auto x: appearancesMiddle)
    //     {
    //         if(x.second > maxCount)
    //         {
    //             maxCount = x.second;
    //             middleBasket = x.first;    
    //         }
    //     }
    // }
    // if(notBlackLeft)
    // {
    //     int maxCount = 0;
    //     for(auto x: appearancesLeft)
    //     {
    //         if(x.second > maxCount)
    //         {
    //             maxCount = x.second;
    //             leftBasket = x.first;    
    //         }
    //     }
    // }

    // laundryBaskets[BASKET_MIDDLE] = middleBasket;
    // laundryBaskets[BASKET_LEFT] = leftBasket;
}

void end_task(intptr_t unused)
{
    grabber.setMode(REGULATED);
    grabber.moveUnlimited(700, true);
    tslp_tsk(50);
    while(grabber.getCurrentSpeed() > 350)
    {
        grabber.moveUnlimited(700);
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


    // resetLifo();
    // lifo.setPIDparams(KP * 1.2, slowKI * 0.7, KD*1.5, 1);
    // lifo.distance(robot.cmToTacho(30), 10, NONE);
    // setLifoSlow();
    // lifo.setAccelParams(150, 20, 20);
    // lifo.distance(20, 3, NONE);
    // lifo.lines(20, 1, NONE);

    // //     // btnEnter.waitForClick();


    // robot.setMode(CONTROLLED);
    // robot.setLinearAccelParams(100, 20, 0);
    // robot.straight(20, 4, COAST);
    // // btnEnter.waitForClick();

    // robot.setLinearAccelParams(100, 20, 20);
    // robot.arc(45, 180, -19/2.0, COAST);
    // // robot.arc(45, 180, -21, COAST);


    // // robot.setMode(CONTROLLED);
    // // robot.setLinearAccelParams(100, 0, 0);
    // // robot.straight(40, 84, COAST);
    // // robot.straight(-40, 84, COAST);

    // btnEnter.waitForClick();

    // grabber.stop();

    // leftScanner.setNormalisation(false);
    // rightScanner.setNormalisation(false);
    // display.resetScreen();
    // while(true)
    // {
    //     colorspaceRGB l = leftScanner.getRGB();
    //     colorspaceRGB r = rightScanner.getRGB();
    //     format(bt, "L: R: %  G: %  B: %  W: %  \nR: R: %  G: %  B: %  W: %  \n") %l.red %l.green %l.blue %l.white %r.red %r.green %r.blue %r.white;
    //     colorspaceHSV l2 = leftScanner.getHSV();
    //     colorspaceHSV r2 = rightScanner.getHSV();
    //     // format(bt, "L: H: %  S: %  V: %  \nR: H: %  S: %  V: %  \n\n") %l2.hue %l2.saturation %l2.value %r2.hue %r2.saturation %r2.value;
    //     display.format("L: %  \nR: %  \n\n\n") %static_cast<int>(scanLaundryBasket(leftScanner)) %static_cast<int>(scanLaundryBasket(rightScanner));
    //     tslp_tsk(10);
    // }


    // leftMotor.setMode(UNREGULATED);
    // rightMotor.setMode(UNREGULATED);
    // leftMotor.setUnregulatedDPS();
    // rightMotor.setUnregulatedDPS();

    // control leftController;
    // control rightController;

    // leftController.setAbsoluteLimits(1150, 8000, 1150);
    // leftController.setPID(10, 1.2, 2, 0.002);
    // leftController.setTargetTolerance(50, 5);

    // rightController.setAbsoluteLimits(1150, 8000, 1150);
    // rightController.setPID(10, 1.2, 2, 0.002);
    // rightController.setTargetTolerance(50, 5);

    // trajectory trj;
    // trj.setLimits(50, 400);
    // trj.makePositionBased(45, 100, 100, 10, 10);

    // timer tim;
    // double time, pos, vel, ac;

    // double Kp = 0.1;
    // double error, result, leftResult, rightResult;
    // double leftTargetPos, rightTargetPos, leftTargetSpeed, rightTargetSpeed;

    // bool isDone = false;

    // while(!isDone)
    // {
    //     time = tim.secElapsed();
    //     trj.getReference(time, &pos, &vel, &ac);
    //     isDone = robot.getPosition() > pos;
    //     leftTargetPos = rightTargetPos = pos = robot.cmToTacho(pos);
    //     leftTargetSpeed = rightTargetSpeed = vel = robot.cmToTacho(vel);
       
    //     error = rightTargetSpeed * leftMotor.getTachoCount() - leftTargetSpeed * rightMotor.getTachoCount();
    //     result = error * Kp;
    //     leftResult = leftTargetSpeed - sign(rightTargetSpeed) * result;
    //     rightResult = rightTargetSpeed + sign(leftTargetSpeed) * result;

    //     leftResult = leftController.updateManual(time, leftMotor.getTachoCount(), leftMotor.getCurrentSpeed(), leftTargetPos, leftResult);
    //     rightResult = rightController.updateManual(time, rightMotor.getTachoCount(), rightMotor.getCurrentSpeed(), rightTargetPos, rightResult);

    //     leftMotor.moveUnlimited(leftResult);
    //     rightMotor.moveUnlimited(rightResult);
    // }

    // leftMotor.stop(BRAKE);
    // rightMotor.stop(BRAKE);


    // btnEnter.waitForClick();



    // resetLifo();
    // lifo.setPIDparams(KP * 1.2, slowKI * 0.7, KD*1.5, 1);
    // lifo.distance(robot.cmToTacho(30), 10, NONE);
    // setLifoSlow();
    // lifo.setAccelParams(150, 20, 20);
    // lifo.distance(20, 5, NONE);
    // lifo.setSensorMode(WHITE_RGB);
    // lifo.unlimited(20, true);
    // bool isDone = false;
    // colorspaceRGB lCur, lPrev, rCur, rPrev;
    // timer conditionTimer;
    // lCur = lPrev = leftSensor.getRGB();
    // rCur = rPrev = rightSensor.getRGB();
    // while(!isDone)
    // {
    //     lifo.unlimited(20);
    //     if(conditionTimer.secElapsed() > 0.01)
    //     {
    //         lCur = leftSensor.getRGB();
    //         rCur = rightSensor.getRGB();

    //         isDone = abs((lCur.red + rCur.red) - (lPrev.red + rPrev.red)) > 30
    //             || abs((lCur.green + rCur.green) - (lPrev.green + rPrev.green)) > 30
    //             || abs((lCur.blue + rCur.blue) - (lPrev.blue + rPrev.blue)) > 30;

    //         lPrev = lCur;
    //         rPrev = rCur;
    //         conditionTimer.reset();
    //     }
    // }

    // robot.setMode(CONTROLLED);
    // robot.setLinearAccelParams(100, 0, 0);
    // robot.arc(45, 45, 8.5, COAST);
    // robot.setLinearAccelParams(100, 0, 45);
    // robot.arcUnlim(45, -8.5, FORWARD, true);
    // colors laundry = WHITE;
    // map<colors, int> appearances;
    // colors current;
    // bool notWhite = false;
    // while(abs(robot.getAngle()) < 50)
    // {
    //     if((current = scanLaundryBlock(rightScanner)) != WHITE)
    //     {
    //         appearances[current]++;
    //         notWhite = true;
    //     }
    //     robot.arcUnlim(45, -8.5, FORWARD);
    // }
    // if(notWhite)
    // {
    //     int maxCount = 0;
    //     for(auto x: appearances)
    //     {
    //         if(x.second > maxCount)
    //         {
    //             maxCount = x.second;
    //             laundry = x.first;    
    //         }
    //     }
    // }
    
    // robot.setLinearAccelParams(100, 45, 0);
    // robot.arc(45, 45, -8.5, COAST);


    // robot.setLinearAccelParams(100, 0, 0);
    // robot.arc(45, -140, -3, COAST);
    

    // display.format("%  \n")%static_cast<int>(laundry);

    // btnEnter.waitForClick();

    // //Mission Code
    // startProcedure();

    // fullRouteStandard(W);
    // pickWater();

    // grabber.stop();

    // rampQueue.push(items::BOTTLE);
    // rampQueue.push(items::BOTTLE);
    // currentPos = BR;
    // currentDirection = SOUTH;

    // fullRouteStandard(YR);

    // rooms[YELLOW].setTask(WHITE);
    // rooms[YELLOW].executeAllActions();


    // grabber.stop();
    // ramp.stop();

    // while(true)
    // {
    //     ramp.setMode(REGULATED);
    //     ramp.moveUnlimited(-1000, true);
    //     tslp_tsk(50);
    //     while(abs(ramp.getCurrentSpeed()) > 400)
    //     {
    //         ramp.moveUnlimited(-1000);
    //         tslp_tsk(1);
    //     }
    //     ramp.stop(BRAKE);
    //     btnEnter.waitForClick();
    //     ramp.setMode(CONTROLLED);
    //     ramp.setUnregulatedDPS();
    //     ramp.setAccelParams(2000, 800, 0);
    //     ramp.moveDegrees(600, 260, COAST);
    //     // btnEnter.waitForClick();
    //     t.secDelay(0.2);
    //     // btnEnter.waitForClick();
    // }


    startProcedure();

    fullRouteStandard(W);
    pickWater();


    fullRouteStandard(GR);
    rooms[GREEN].executeAllActions();
    fullRouteStandard(RR);
    rooms[RED].executeAllActions();
    fullRouteStandard(BR);
    rooms[BLUE].executeAllActions();
    fullRouteStandard(YR);
    rooms[YELLOW].executeAllActions();
    fullRouteStandard(L);

    scanLaundryBaskets();
    leaveLaundry();

    fullRouteStandard(S);
    finishProcedure();

    // grabber.stop(COAST);

    // resetLifo();
    // lifo.setPIDparams(KP * 1.2, slowKI * 0.7, KD*1.5, 1);
    // lifo.distance(robot.cmToTacho(30), 10, NONE);
    // setLifoSlow();
    // lifo.setAccelParams(150, 20, 20);
    // lifo.distance(20, 3, NONE);
    // lifo.lines(20, 1, NONE);

    // scanStage = 1;
    // act_tsk(BASKET_SCAN_TASK);
    // tslp_tsk(1);

    // robot.setLinearAccelParams(100, 20, 0);
    // robot.straight(20, 1, COAST);
    // robot.setLinearAccelParams(100, 0, 0);
    // robot.arc(35, 90, 0, COAST);
    // scanStage = 2;
    // robot.straight(35, 4, COAST);
    // scanStage = 3;
    // robot.straight(35, -4, COAST);
    // robot.arc(35, 98, 0, COAST);
    // robot.setLinearAccelParams(100, 0, 0);
    // robot.straight(35, 2, COAST);

    // // btnEnter.waitForClick();

    // colors temp[2];
    // temp[0] = laundryBaskets[BASKET_LEFT];
    // temp[1] = laundryBaskets[BASKET_MIDDLE];
    // laundryBaskets[BASKET_RIGHT] = findTheLastColor(temp, 3);  


    // DEBUGPRINT("First laundry basket was scanned and it has the color: ");
    // switch(laundryBaskets[BASKET_LEFT])
    // {
    //     case RED:
    //         DEBUGPRINT("red.\n");
    //         break;
    //     case BLACK:
    //         DEBUGPRINT("black.\n");
    //         break;
    //     case YELLOW:
    //         DEBUGPRINT("yellow.\n");
    //         break;
    // }

    // DEBUGPRINT("Second laundry basket was scanned and it has the color: ");
    // switch(laundryBaskets[BASKET_MIDDLE])
    // {
    //     case RED:
    //         DEBUGPRINT("red.\n");
    //         break;
    //     case BLACK:
    //         DEBUGPRINT("black.\n");
    //         break;
    //     case YELLOW:
    //         DEBUGPRINT("yellow.\n");
    //         break;
    // }
    
    // DEBUGPRINT("Last laundry basket was scanned and it has the color: ");
    // switch(laundryBaskets[BASKET_RIGHT])
    // {
    //     case RED:
    //         DEBUGPRINT("red.\n");
    //         break;
    //     case BLACK:
    //         DEBUGPRINT("black.\n");
    //         break;
    //     case YELLOW:
    //         DEBUGPRINT("yellow.\n");
    //         break;
    // }


    // rampQueue.push(items::LAUNDRY_BLACK);
    // rampQueue.push(items::LAUNDRY_RED);
    // rampQueue.push(items::LAUNDRY_YELLOW);
    // scanLaundryBaskets();    

    // robot.setLinearAccelParams(100, 0, 0);
    // robot.straight(30, -4, COAST);
    // emptyRampLaundry();
    // robot.straight(30, 4, COAST);

    // robot.arc(35, -36, 0, COAST); //32

    // robot.straight(30, -5, COAST);
    // emptyRampLaundry();
    // robot.straight(30, 5, COAST);

    // robot.arc(35, 72, 0, COAST); //32

    // robot.straight(30, -5, COAST);
    // emptyRampLaundry();
    // robot.straight(30, 5, COAST);

    // robot.arc(35, -36, 0, COAST); //32

    // robot.straight(30, -4, COAST);
    // emptyRampLaundry();
    // robot.straight(30, 4, COAST);

    // leaveLaundry();
    // L_S(SOUTH);
    // finishProcedure();
     


    // robot.setLinearAccelParams(100, 0, 0);
    // scanLaundryBaskets();
    // turnToBasket(BASKET_MIDDLE, BASKET_LEFT);
    // robot.straight(35, -4, COAST);
    // emptyRampLaundry();
    // robot.straight(35, 4, COAST);
    // turnToBasket(BASKET_LEFT, BASKET_MIDDLE);
    // robot.straight(35, -2, COAST);
    // emptyRampLaundry();
    // robot.straight(35, 2, COAST);
    // turnToBasket(BASKET_MIDDLE, BASKET_RIGHT);
    // robot.straight(35, -4, COAST);
    // emptyRampLaundry();
    // robot.straight(35, 4, COAST);
    // turnToBasket(BASKET_RIGHT, BASKET_MIDDLE);



    // rampQueue.push(BOTTLE);
    // rampQueue.push(BOTTLE);
    // grabber.stop(BRAKE);

    // resetLifo();
    // lifo.setPIDparams(KP * 1.2, slowKI * 0.7, KD*1.5, 1);
    // lifo.distance(robot.cmToTacho(30), 10, NONE);
    // setLifoSlow();
    // lifo.setAccelParams(150, 20, 20);
    // lifo.distance(20, 3, NONE);
    // lifo.lines(20, 1, NONE);

    // //     // btnEnter.waitForClick();

    

    // robot.setMode(CONTROLLED);
    // correctionBeforeMovement();
    // robot.setLinearAccelParams(150, 10, 20);
    // robot.straight(20, 5, NONE);
    // correctionOnTheMove();
    // robot.setLinearAccelParams(150, 20, 10);
    // robot.straight(35, 50, BRAKE);


    // robot.stop(BRAKE);
    // btnEnter.waitForClick();

    // currentPos = YR;
    // currentDirection = SOUTH;
    // fullRouteStandard(L);



    // rooms[GREEN].setTask(GREEN);
    // fullRouteStandard(GR);
    // rooms[GREEN].executeAllActions();

    // fullRouteStandard(RR);
    // rooms[BLUE].setTask(GREEN);
    // rooms[BLUE].executeAllActions();

    // fullRouteStandard(CR);

   

    robot.stop(BRAKE);



    
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
