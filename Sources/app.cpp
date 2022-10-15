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
chassis robot(&leftMotor, &rightMotor, 6.24, 17, 0.1, 0.007, 0);
colorSensor leftSensor(SensorPort::S2, false, "WRO2022");
colorSensor rightSensor(SensorPort::S3, false, "WRO2022");
colorSensor leftScanner(SensorPort::S1, false, "WRO2022");
colorSensor rightScanner(SensorPort::S4, false, "WRO2022");
lineFollower lifo(700, &robot, &leftSensor, &rightSensor);
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
        lineDetector->getRGB();
        if((current = scanLaundryBlock(*scanner)) != NO_COLOR)
            appearances[current]++;
    }

    current = analyzeFrequency(appearances, NO_COLOR);
    if(current != RED && current != YELLOW && current != NO_COLOR) current = BLACK;
    scannedValue = current;
    roomScanStage = 4;  //final distance based on room 
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
    // leftScanner.setNormalisation(false);
    // rightScanner.setNormalisation(false);
    // display.resetScreen();
    // while(true)
    // {
    //     // colorspaceRGB l = leftScanner.getRGB();
    //     // colorspaceRGB r = rightScanner.getRGB();
    //     // format(bt, "L: R: %  G: %  B: %  W: %  \nR: R: %  G: %  B: %  W: %  \n") %l.red %l.green %l.blue %l.white %r.red %r.green %r.blue %r.white;
    //     // colorspaceHSV l2 = leftScanner.getHSV();
    //     // colorspaceHSV r2 = rightScanner.getHSV();
    //     // format(bt, "L: H: %  S: %  V: %  \nR: H: %  S: %  V: %  \n\n") %l2.hue %l2.saturation %l2.value %r2.hue %r2.saturation %r2.value;
    //     display.format("L: %  \nR: %  \n\n\n") %static_cast<int>(scanCodeBlock(leftScanner)) %static_cast<int>(scanCodeBlock(rightScanner));
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


    lifo.setDoubleFollowMode("SL", "SR");
    lifo.initializeMotionMode(UNREGULATED);
    robot.setUnregulatedDPS(true);
    lifo.setSensorMode(REFLECTED);
    lifo.addPIDparams(30, 4, 1, 30);    //30 speed
    lifo.addPIDparams(40, 4, 3, 40);    //40 speed
    lifo.addPIDparams(50, 5, 4, 40);    //50 speed
    lifo.addPIDparams(60, 6, 5, 60);    //60 speed

    // // lineFollower lifoFinisher(400, &robot, &leftSensor, &rightSensor);
    // // lifoFinisher.setDoubleFollowMode("SL", "SR");
    // // lifoFinisher.initializeMotionMode(CONTROLLED);
    // // lifoFinisher.setSensorMode(REFLECTED);

    // while(true)
    // {
    //     // // lifo.setPIDparams(3, 3, 120);    //Extreme correction 30speed
        
    //     // // lifo.distance(30, 5, NONE);

    //     // // // lifo.setPIDparams(4, 0.5, 60);   //About normal 30speed
    //     // // lifo.setPIDparams(4, 1.5, 80);   //About normal 40speed
    //     // // lifo.lines(40, 1, NONE, 8.5, true);

    //     // lifo.setPIDparams(3, 3, 120);    //Extreme correction 30speed
    //     // // lifo.distance(40, 3, NONE);

    //     // lifo.setPIDparams(4, 1.5, 80);   //About normal 40speed
    //     // lifo.distance(40, 13, NONE);


    //     // // lifo.setPIDparams(3, 0, 30);    //Finisher 30speed
    //     // lifo.setPIDparams(4, 0, 40);    //Finisher 40speed
        
    //     lifo.setPIDparams(3, 3, 120);    //Extreme correction 30speed
    //     lifo.distance(30, 5, NONE);

    //     lifo.setPIDparams(4, 1.5, 80);   //About normal 40speed
    //     lifo.distance(40, 8, NONE);


    //     //Get in the room
    //     //Task setup
    //     scanner = &rightScanner;
    //     lineDetector = &rightSensor;
    //     roomScanStage = 1;
    //     act_tsk(ROOM_ENTRANCE_TASK);
    //     tslp_tsk(1);

    //     robot.setMode(CONTROLLED);
    //     robot.setLinearAccelParams(100, 40, 40);
    //     robot.straightUnlim(40, true);
    //     while(roomScanStage != 4)
    //     {
    //         robot.straightUnlim(40);
    //         tslp_tsk(1);
    //     }

    //     robot.setLinearAccelParams(100, 40, 0);
    //     robot.straight(40, 6.6, COAST);

    //     display.resetScreen();
    //     display.format("%  \n") %static_cast<int>(scannedValue);


    //     btnEnter.waitForClick();
    // }


    // startProcedure();

    // fullRouteStandard(W);

    // pickWater();

    currentPos = W;
    rampQueue.push(BOTTLE);
    rampQueue.push(BOTTLE);
    act_tsk(CLOSE_RAMP_TASK);
    tslp_tsk(1);
    lifo.setPIDparams(3, 3, 120);    //Extreme correction 30speed
    lifo.distance(30, 7, NONE);
    lifo.setPIDparams(4, 1.5, 80);   //About normal 40speed
    lifo.distance(40, 3, NONE);
    lifo.lines(40, 1, NONE);
    robot.setMode(CONTROLLED);
    robot.setLinearAccelParams(100, 40, 40);
    robot.straight(40, 0.5, COAST);
    
    fullRouteStandard(G);
    rooms[GREEN].executeAllActions();
    fullRouteStandard(R);
    rooms[RED].executeAllActions();

    lifo.setDoubleFollowMode("SL", "70");

    lifo.setPIDparams(3, 3, 120);    //Extreme correction 30speed
    lifo.distance(30, 5, NONE);

    lifo.setPIDparams(4, 1.5, 80);   //About normal 40speed
    lifo.lines(40, 1, NONE, 8.5);

    robot.setMode(CONTROLLED);
    robot.setLinearAccelParams(100, 40, 50);
    robot.arc(50, 40, 8.5, NONE); 
    robot.setLinearAccelParams(100, 50, 40);
    robot.arc(50, 65, 18, NONE);
    robot.setLinearAccelParams(100, 40, 40);
    robot.arcUnlim(40, 18, FORWARD, true);
    while(rightSensor.getReflected() < 60)
        robot.arcUnlim(40, 18, FORWARD);
    robot.stop(COAST);

    lifo.setDoubleFollowMode("70", "SR");

    lifo.setPIDparams(3, 3, 120);    //Extreme correction 30speed
    lifo.distance(30, 5, NONE);

    lifo.setPIDparams(4, 1.5, 80);   //About normal 40speed
    lifo.lines(40, 1, NONE, 8.5, true);

    lifo.distance(40, 10, COAST);

    // rooms[GREEN].executeAllActions();
    // fullRouteStandard(R);
    // rooms[RED].executeAllActions();
    // fullRouteStandard(B);
    // rooms[BLUE].executeAllActions();
    // fullRouteStandard(Y);
    // rooms[YELLOW].executeAllActions();
    // fullRouteStandard(L);

    // scanLaundryBaskets();
    // leaveLaundry();

    // fullRouteStandard(S);
    // finishProcedure();

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
