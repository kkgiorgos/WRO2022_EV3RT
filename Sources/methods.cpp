#include "methods.h"
#include "ev3ys.h"
#include "tasks.h"
#include "globalRobot.h"
#include <cmath>

using namespace std;
using namespace ev3ys;
using namespace ev3cxx;

void setLifo(const char *leftPos, const char *rightPos)
{
    lifoControlled.setDoubleFollowMode(leftPos, rightPos);
    lifoUnregNormal.setDoubleFollowMode(leftPos, rightPos);
    lifoUnregExtreme.setDoubleFollowMode(leftPos, rightPos);
}

void align(double time, bool stop)
{
    timer t;
    speedMode prevMode = robot.getMode();
    double kp = 2;
    while(t.secElapsed() < time)
    {
        int error = leftSensor.getReflected() - rightSensor.getReflected();
        leftMotor.moveUnlimited(error * kp);
        rightMotor.moveUnlimited(error * kp * -1);
        //tslp_tsk(1);
        t.secDelay(0.001);
    }
    robot.stop(stop ? BRAKE : COAST);
    robot.setMode(prevMode);
}

void alignPerpendicular(double time, bool stop)
{
    timer t;
    speedMode prevMode = robot.getMode();
    double kp = -0.5;
    int leftTarget, rightTarget;
    robot.setMode(speedMode::UNREGULATED);

    robot.tankUnlim(-40, -40, true);
    int leftMin = 100, leftMax = 0, rightMin = 100, rightMax = 0;
    while(t.secElapsed() < 0.3)
    {
        int left = leftSensor.getReflected();
        int right = rightSensor.getReflected();

        if(left < leftMin)
            leftMin = left;
        else if(left > leftMax)
            leftMax = left;

        if(right < rightMin)
            rightMin = right;
        else if(right > rightMax)
            rightMax = right;

        robot.tankUnlim(-40, -40);
    }

    leftTarget = (leftMin + leftMax) / 2;
    rightTarget = (rightMin + rightMax) / 2;
    t.reset();
    while(t.secElapsed() < time)
    {
        int leftError = leftSensor.getReflected() - leftTarget;
        int rightError = rightSensor.getReflected() - rightTarget;
        robot.tankUnlim(leftError * kp, rightError * kp);
        t.secDelay(0.001);
    }
    robot.stop(stop ? BRAKE : COAST);
    robot.setMode(prevMode);
}

void alignOnMove(double speed) //This will change the robot mode to CONTROLLED
{
    speedMode lastMode = robot.getMode();
    robot.setMode(speedMode::REGULATED);
    robot.tankUnlim(robot.cmToTacho(speed), robot.cmToTacho(speed), true);

    rightSensor.getReflected();
    leftSensor.getReflected();
    while(!rightSensor.getLineDetected() && !leftSensor.getLineDetected())
    {
        rightSensor.getReflected();
        leftSensor.getReflected();
    }
    bool isRight = rightSensor.getLineDetected();

    rightSensor.getReflected();
    leftSensor.getReflected();
    robot.resetPosition();
    if(isRight)
        while(!leftSensor.getLineDetected())
            leftSensor.getReflected();
    else
        while(!rightSensor.getLineDetected())
            rightSensor.getReflected();

    double length = robot.getPosition();
    double sensDiff = 2.5;
    double angle = atan(length / sensDiff) * (180 / MATH_PI);
    double radius = (isRight) ? 9 : -9;
    robot.arc(robot.cmToTacho(speed), angle, radius, NONE);
}

void emptyRampLaundry()
{
    ramp.setMode(CONTROLLED);
    ramp.setUnregulatedDPS();
    ramp.setAccelParams(2000, 800, 0);
    ramp.moveDegrees(600, 260, COAST);
    timer::secDelay(0.1);
    act_tsk(CLOSE_RAMP_TASK);
    tslp_tsk(1);
}

void emptyRampWaterStage1(bool wait)
{
    ramp.moveDegrees(800, 220, BRAKE, wait);
}
void emptyRampWaterStage2()
{
    ramp.moveUnlimited(500, true);
    tslp_tsk(50);
    while(abs(ramp.getCurrentSpeed()) > 200)
    {
        ramp.moveUnlimited(500);
        tslp_tsk(1);
    }
    ramp.stop(BRAKE);   
}

void reverse(lifoRobotPosition startAlignment, lifoRobotPosition endAlignment, ev3ys::breakMode stopMode)
{
    robot.setMode(CONTROLLED);
    robot.setLinearAccelParams(100, 0, 0);

    double arcCenter = 0;
    if(startAlignment == LEFT_OF_LINE) arcCenter += 1;
    if(startAlignment == RIGHT_OF_LINE) arcCenter -= 1;
    if(endAlignment == LEFT_OF_LINE) arcCenter += 1;
    if(endAlignment == RIGHT_OF_LINE) arcCenter -= 1;

    robot.arc(45, 180, arcCenter, stopMode);
}

void leftTurn(lifoRobotPosition endAlignment, ev3ys::breakMode stopMode)
{
    robot.setMode(CONTROLLED);
    robot.setLinearAccelParams(100, 0, 0);
    
    double arcCenter;
    if(endAlignment == CENTERED) arcCenter = -5.5;
    else if(endAlignment == LEFT_OF_LINE) arcCenter = -3.5;
    else arcCenter = -7.5;

    robot.arc(45, 90, arcCenter, stopMode);
}

void rightTurn(lifoRobotPosition endAlignment, ev3ys::breakMode stopMode)
{
    robot.setMode(CONTROLLED);
    robot.setLinearAccelParams(100, 0, 0);

    double arcCenter;
    if(endAlignment == CENTERED) arcCenter = 5.5;
    else if(endAlignment == LEFT_OF_LINE) arcCenter = 7.5;
    else arcCenter = 3.5;

    robot.arc(45, 90, arcCenter, stopMode);
}

colors lifoRoute1Line(lifoRobotPosition alignment, double totalDistance, double extremePhase, double slowPhase, double controlledPhase, double maxSpeed, lineDetectionMode detectLine, const char* lifoTarget, ev3ys::breakMode stopMode)
{
    //Set Lifo Params
    double normalPhase = totalDistance - extremePhase - slowPhase - controlledPhase;

    if(alignment != OTHER)
        currentAlignment = alignment;

    if(alignment == LEFT_OF_LINE)
        setLifo("SR", lifoTarget);
    else if(alignment == RIGHT_OF_LINE)
        setLifo(lifoTarget, "SL");
    else if(alignment == CENTERED)
        setLifo("SL", "SR");
    //Follow Extreme 30 speed
    if(extremePhase != 0)
        lifoUnregExtreme.distance(30, extremePhase, NONE);
    //Follow Normal at given speed
    if(normalPhase > 0)
        lifoUnregNormal.distance(maxSpeed, normalPhase, NONE);
    
    if(controlledPhase == 0 && slowPhase > 0)
    {
        //Follow Normal slow (30) until line
        if(detectLine == NO_DETECT)
            lifoUnregNormal.distance(30, slowPhase, stopMode);
        else if(detectLine == NORMAL)
            lifoUnregNormal.lines(30, 1, stopMode, slowPhase);
        else if(detectLine == COLORED)
        {
            lifoUnregNormal.setSensorMode(WHITE_RGB);
            lifoUnregNormal.lines(30, 1, stopMode, slowPhase);
            lifoUnregNormal.setSensorMode(REFLECTED);
        }
    }   
    else
    {
        //Follow Normal slow (30)
        if(slowPhase > 0)
            lifoUnregNormal.distance(30, slowPhase, NONE);
        //Switch to CONTROLLED 30 speed until line
        if(detectLine == NO_DETECT)
            lifoControlled.distance(30, controlledPhase, stopMode);
        else if(detectLine == NORMAL)
            lifoControlled.lines(30, 1, stopMode, controlledPhase);
        else if(detectLine == COLORED)
        {
            lifoControlled.setSensorMode(WHITE_RGB);
            lifoControlled.lines(30, 1, stopMode, controlledPhase);
            lifoControlled.setSensorMode(REFLECTED);
        }
    }

    if(detectLine == SCANNER)
    {
        if(controlledPhase != 0)
        {
            lifoControlled.distance(30, controlledPhase, NONE);
            stopScanning = false;
            act_tsk(HUMAN_SCAN_TASK);
            tslp_tsk(1);
            leftSensor.resetFiltering();
            rightSensor.resetFiltering();
            lifoControlled.unlimited(30, true);
            while(scannedValue == NO_COLOR && !lifoControlled.getLineDetected() && robot.getPosition () < 5)            
            {
                lifoControlled.unlimited(30, false);
            }
            robot.resetPosition();
            while(robot.getPosition() < 1 && !lifoControlled.getLineDetected())            
            {
                lifoControlled.unlimited(30, false);
            }
            stopScanning = true;
            // lifoControlled.stop(stopMode);
        }
        else
        {
            lifoUnregNormal.distance(30, slowPhase, NONE);
            stopScanning = false;
            scannedValue = NO_COLOR;
            act_tsk(HUMAN_SCAN_TASK);
            tslp_tsk(1);
            leftSensor.resetFiltering();
            rightSensor.resetFiltering();
            lifoUnregNormal.unlimited(30, true);
            while(scannedValue == NO_COLOR && !lifoUnregNormal.getLineDetected() && robot.getPosition () < 5)            
            {
                lifoUnregNormal.unlimited(30, false);
            }
            robot.resetPosition();
            while(robot.getPosition() < 1 && !lifoUnregNormal.getLineDetected())            
            {
                lifoUnregNormal.unlimited(30, false);
            }
            stopScanning = true;
            // lifoUnregNormal.stop(stopMode);
        }
    }

    robot.stop(stopMode);
}

void switchLifoRobotPosition(double speed, lifoRobotPosition startAlignment, lifoRobotPosition endAlignment)
{
    if(startAlignment == CENTERED && endAlignment != CENTERED)
    {
        robot.setMode(CONTROLLED);
        robot.setLinearAccelParams(100, speed, speed);
        robot.arc(speed, 15, (endAlignment == LEFT_OF_LINE) ? -20 : 20, NONE);
    }
    if(startAlignment == RIGHT_OF_LINE && endAlignment == CENTERED)
    {
        robot.setMode(CONTROLLED);
        robot.setLinearAccelParams(100, speed, speed);
        robot.arc(speed, 15, -20, NONE);
    }
    if(startAlignment == LEFT_OF_LINE && endAlignment == CENTERED)
    {
        robot.setMode(CONTROLLED);
        robot.setLinearAccelParams(100, speed, speed);
        robot.arc(speed, 15, 20, NONE);
    }
    currentAlignment = endAlignment;
}
