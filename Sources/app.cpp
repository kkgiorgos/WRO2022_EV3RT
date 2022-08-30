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
motor leftMotor(MotorPort::B, true);
motor rightMotor(MotorPort::C, false);
chassis robot(&leftMotor, &rightMotor, 6.24, 18, 0.1, 0.01);
colorSensor leftSensor(SensorPort::S2, false, "WRO2022");
colorSensor rightSensor(SensorPort::S3, false, "WRO2022");
colorSensor leftScanner(SensorPort::S1, false, "WRO2022");
colorSensor rightScanner(SensorPort::S4, false, "WRO2022");
lineFollower lifo(400, &robot, &leftSensor, &rightSensor);


queue<items, list<items>> rampQueue;
colors laundryBaskets[3];
map<colors, room> rooms;

double KP = 0.5;  //OLD: 2
double KI = 0.05;  //OLD: 2
double KD = 5;//OLD: 200
double PIDspeed = 75;

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
    ramp.moveUntilStalled(-300, BRAKE);
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

    // currentPos = GR;
    // currentDirection = NORTH;
    // fullRouteStandard(RR);

    // rooms[RED].setTask(WHITE);
    // rooms[RED].enterRoom();

    // robot.setMode(REGULATED);
    // robot.arc(800, -90, 2.8, BRAKE);

    // display.resetScreen();
    // while(true)
    // {
    //     colorspaceHSV left = leftSensor.getHSV();
    //     colorspaceHSV right = rightSensor.getHSV();
    //     tslp_tsk(10);
    //     display.format("L\nH:%  \nS:%  \nV:%  \nR\nH:%  \nS:%  \nV:%  \n")%left.hue %left.saturation %left.value %right.hue %right.saturation %right.value;
    // }
    
    // currentPos = BR;
    // currentDirection = NORTH;
    // fullRouteStandard(YR);

    // rooms[YELLOW].setTask(WHITE);
    // rooms[YELLOW].executeAllActions();

    // fullRouteStandard(L);

    // robot.setMode(REGULATED);
    // robot.arc(800, -87, -3, BRAKE);

    // openGrabber();
    // robot.setLinearAccelParams(200, 0, 0);
    // robot.straight(35, 10);
    // robot.straight(-10, 1);
    // pickBlock();
    // robot.straight(-40, 9);

    // robot.setMode(CONTROLLED);
    // robot.setAngularAccelParams(1000, 0, 50);
    // robot.turn(500, -52);

    // robot.setMode(REGULATED);
    // robot.arc(800, 35, 3, BRAKE);

    // openGrabber();
    // robot.setMode(CONTROLLED);
    // robot.setLinearAccelParams(100, 0, 0);
    // robot.straight(50, 11.5);
    // grabber.moveDegrees(140, 100, breakMode::BRAKE);

    // robot.setMode(CONTROLLED);
    // robot.setAngularAccelParams(1000, 0, 50);    
    // robot.turn(500, -113);

    // robot.setMode(CONTROLLED);
    // robot.setLinearAccelParams(200, 0, 0);
    // robot.straight(50, 15);

    // grabber.moveUntilStalled(120);


    // fullRouteStandard(RR);

    // rooms[RED].setTask(WHITE);
    // rooms[RED].executeAllActions();

    // setLifoRight();
    // lifo.setAlignMode(true);
    // lifo.initializeMotionMode(UNREGULATED);
    // lifo.seconds(0, 0.2, NONE);
    // resetLifo();

    // while(true)
    // {  
    //     btnEnter.waitForPress();
    //     timer::secDelay(0.1);
    //     emptyRampWater();
    // }

    // while(true)
    // {  
    //     btnEnter.waitForPress();
    //     openGrabber();
    //     btnEnter.waitForPress();
    //     pickBlock();
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

    /*currentPos = CL;
    currentDirection = WEST;
    //lifo1LineDist(15);
    fullRouteStandard(YR);

    //rooms[YELLOW].enterRoom();

    for(int i = 0; i < 15; i++)
    {
        ramp.moveDegrees(120, 80, BRAKE);
        t.secDelay(0.2);
        ramp.moveUntilStalled(-300, BRAKE);
        btnEnter.waitForPress();
    }*/

    /*display.resetScreen();
    while(true)
    {
        display.format("%  \n%  \n\n\n") %static_cast<int>(scanLaundryBlock(leftScanner)) %static_cast<int>(scanLaundryBlock(rightScanner));
        tslp_tsk(10);
    }*/

    /*lifo.setDoubleFollowMode("66", "SR");
    lifo.setPIDparams(KP*1.75, KI*1.75, KD*1.75, PIDspeed);
    lifo.unlimited(50, true);
    lifo.setAccelParams(600, 50, 50);
    do
    {
        leftSensor.getReflected();
        rightSensor.getReflected();
        lifo.unlimited(50);
    }
    while(!leftSensor.getLineDetected() && !rightSensor.getLineDetected());

    CL_FL(WEST);

    robot.stop(BRAKE);
    robot.setMode(REGULATED);
    robot.arc(800, -87, 2, BRAKE);

    lifo.setDoubleFollowMode("66", "SR");
    lifo.setPIDparams(KP*1.75, KI*1.75, KD*1.75, PIDspeed);
    lifo.setAccelParams(600, 40, 40);
    lifo.unlimited(40, true);
    while(!detectColorLine(leftSensor, BLUE))
        lifo.unlimited(40);

    robot.setMode(CONTROLLED);
    robot.setLinearAccelParams(200, 40, 40);
    robot.straightUnlim(40, true);    
    colors laundryBlue = WHITE;
    map<colors, int> appearances;
    colors current;
    bool notWhite = false;
    while(leftSensor.getReflected() < 80)
    {
        if((current = scanLaundryBlock(leftScanner)) != WHITE)
        {
            appearances[current]++;
            notWhite = true;
        }
        robot.straightUnlim(40);
    }
    if(notWhite)
    {
        int maxCount = 0;
        for(auto x: appearances)
        {
            if(x.second > maxCount)
            {
                maxCount = x.second;
                laundryBlue = x.first;    
            }
        }
    }
    display.resetScreen();
    display.format("%  \n")%static_cast<int>(laundryBlue);
    robot.stop(BRAKE);

    robot.setMode(REGULATED);
    robot.arc(800, -90, 3, BRAKE);*/

    /*display.resetScreen();
    while(true)
    {
        tslp_tsk(10);
        display.format("L:%  \n\n\n\nR:%  \n\n\n\n")%leftSensor.getReflected() %rightSensor.getReflected();
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


    /*lifo.setDoubleFollowMode("66", "SR");
    lifo.setPIDparams(KP*1.75, KI*1.75, KD*1.75, PIDspeed);

    lifo1LineDist(30);*/
    

    //Mission Code
    /*startProcedure();
    fullRouteStandard(W);
    pickWater();
    fullRouteStandard(RR);
    rooms[RED].executeAllActions();
    fullRouteStandard(GR);
    rooms[GREEN].executeAllActions();
    fullRouteStandard(YR);
    rooms[YELLOW].executeAllActions();
    fullRouteStandard(BR);
    rooms[BLUE].executeAllActions();
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
