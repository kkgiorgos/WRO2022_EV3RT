#include "methods.h"
#include "ev3ys.h"
#include "tasks.h"
#include "globalRobot.h"
#include <cmath>

using namespace std;
using namespace ev3ys;
using namespace ev3cxx;

void resetLifo()
{
    // lifo.setDoubleFollowMode("SL", "SR");
    // lifo.setPIDparams(KP , KI , KD , PIDspeed);
    colorCoef = 1;
    // lifo.setAlignMode(false);
    // lifo.initializeMotionMode(CONTROLLED);

    lifo.setDoubleFollowMode("SL", "SR");
    lifo.initializeMotionMode(UNREGULATED);
    lifo.setAlignMode(true);
    robot.setUnregulatedDPS(true);
    lifo.setPIDparams(KP, KI, KD, PIDspeed);
    lifo.setSensorMode(REFLECTED);
    //printf("Reset Color of Line Followed\n");
}

void setLifoNormalReg()
{
    lifo.setDoubleFollowMode("SL", "SR");
    lifo.initializeMotionMode(CONTROLLED);
    lifo.setAlignMode(true);
    lifo.setSensorMode(REFLECTED);
    lifo.setPIDparams(1.5, 3, 150, PIDspeed);
}

void setLifoSlow()
{
    lifo.setDoubleFollowMode("SL", "SR");
    lifo.setPIDparams(slowKP, slowKI, slowKD, PIDspeed);
    lifo.initializeMotionMode(CONTROLLED);
    lifo.setAlignMode(true);
    lifo.setSensorMode(REFLECTED);
}

void setLifoLeft(bool slow)
{
    lifo.setDoubleFollowMode("SL", "70");
    if(slow)
        lifo.setPIDparams(slowKP*1.7, slowKI*1.7, slowKD*2, PIDspeed);
    else
        lifo.setPIDparams(KP*1.7, KI*1.7, KD*2, PIDspeed);
    prevRight = rightSensor.getReflected();
    universalTimer.reset();
}

void setLifoLeftExtreme(bool slow)
{
    lifo.setDoubleFollowMode("SL", "70");
    if(slow)
        lifo.setPIDparams(slowKP*3, slowKI*1.5, slowKD*5, PIDspeed);   
    else
        lifo.setPIDparams(KP*3, KI*1.5, KD*5, PIDspeed);   
    prevRight = rightSensor.getReflected();
    universalTimer.reset();
}

void setLifoRight(bool slow)
{
    lifo.setDoubleFollowMode("70", "SR");
    if(slow)
        lifo.setPIDparams(slowKP*1.7, slowKI*1.7, slowKD*2, PIDspeed);
    else
        lifo.setPIDparams(KP*1.7, KI*1.7, KD*2, PIDspeed);
    prevLeft = leftSensor.getReflected();
    universalTimer.reset();
}

void setLifoRightExtreme(bool slow)
{
    lifo.setDoubleFollowMode("70", "SR");
    if(slow)
        lifo.setPIDparams(slowKP*3, slowKI*1.5, slowKD*5, PIDspeed);   
    else
        lifo.setPIDparams(KP*3, KI*1.5, KD*5, PIDspeed);   
    prevLeft = leftSensor.getReflected();
    universalTimer.reset();
}

void executeLifoLeftUnlim(int velocity)
{
    int refRight = rightSensor.getReflected();
    if(refRight < 25 || refRight > 40)
        lifo.setDoubleFollowMode("N", "N");
    else
        lifo.setDoubleFollowMode("SL", "70");
    lifo.unlimited(velocity);
}

void executeLifoRightUnlim(int velocity)
{
    int refLeft = leftSensor.getReflected();
    if(refLeft < 25 || refLeft > 40)
        lifo.setDoubleFollowMode("N", "N");
    else
        lifo.setDoubleFollowMode("70", "SR");
    lifo.unlimited(velocity);
}

void align(double time, bool stop)
{
    timer t;
    speedMode prevMode = robot.getMode();
    double kp = 2 * colorCoef;
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
    double kp = -0.5 * colorCoef;
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

void correctionBeforeMovement()
{
    speedMode prevMode = robot.getMode();
    robot.setMode(CONTROLLED);
    robot.setAngularAccelParams(600, 200, 200);
    robot.turn(200, 2, NONE);
    robot.setMode(prevMode);
}

void correctionOnTheMove()
{
    robot.tank(robot.cmToTacho(23), robot.cmToTacho(20), robot.cmToTacho(1), NONE);
}

void lifo1WhiteLineLeftSlow(double startVelocity, double distance, double slowVelocity, breakMode stopMode)
{
    lifo.initializeMotionMode(CONTROLLED);
    lifo.setDoubleFollowMode("SL", "70");
    lifo.setAlignMode(true);
    lifo.setPIDparams(slowKP*1.7, slowKI*1.7, slowKD*2, 1);
    lifo.setAccelParams(200, startVelocity, slowVelocity);
    lifo.distance(startVelocity, distance, NONE);
    lifo.unlimited(slowVelocity, true);
    while(rightSensor.getReflected() < 60)
    {
        executeLifoLeftUnlim(slowVelocity);
    }
    robot.stop(stopMode);
}

void lifo1WhiteLineRightSlow(double startVelocity, double distance, double slowVelocity, breakMode stopMode)
{
    lifo.initializeMotionMode(CONTROLLED);
    lifo.setDoubleFollowMode("70", "SR");
    lifo.setAlignMode(true);
    lifo.setPIDparams(slowKP*1.7, slowKI*1.7, slowKD*2, 1);
    lifo.setAccelParams(200, startVelocity, slowVelocity);
    lifo.distance(startVelocity, distance, NONE);
    lifo.unlimited(slowVelocity, true);
    while(leftSensor.getReflected() < 60)
    {
        executeLifoRightUnlim(slowVelocity);
    }
    robot.stop(stopMode);
}

void emptyRampLaundry()
{
    ramp.setMode(CONTROLLED);
    ramp.setUnregulatedDPS();
    ramp.setAccelParams(2000, 800, 0);
    ramp.moveDegrees(600, 260, COAST);
    timer::secDelay(0.2);
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
    robot.setLinearAccelParams(100, 10, 20);
    
    double arcCenter;
    if(endAlignment == CENTERED) arcCenter = -5.5;
    else if(endAlignment == LEFT_OF_LINE) arcCenter = -3.5;
    else arcCenter = -7.5;

    robot.arc(45, 90, arcCenter, stopMode);
}

void rightTurn(lifoRobotPosition endAlignment, ev3ys::breakMode stopMode)
{
    robot.setMode(CONTROLLED);
    robot.setLinearAccelParams(100, 10, 20);

    double arcCenter;
    if(endAlignment == CENTERED) arcCenter = 5.5;
    else if(endAlignment == LEFT_OF_LINE) arcCenter = 7.5;
    else arcCenter = 3.5;

    robot.arc(45, 90, arcCenter, stopMode);
}

void lifo1LineDist(lifoRobotPosition alignment, double totalDistance, double startPhaseDist, double endPhaseDist, double slowDist, lineDetectionMode detectLine, ev3ys::breakMode stopMode)
{
    timer t;
    double speed, coef;
    resetLifo();
    if(alignment == CENTERED)
        coef = 1;
        // lifo.setPIDparams(KP * 1.2, KI * 0.7, KD*1.5, 1);
    else //ONE SENSOR EDGE FOLLOWING
        coef = 1.8;
        // lifo.setPIDparams(KP * 1.2 * 1.6, KI * 0.7 * 1.6, KD*1.5 * 1.6, 1);
    lifo.setPIDparams(3 * coef, 3 * coef, 120 * coef, PIDspeed);
    if(alignment == LEFT_OF_LINE)
        lifo.setDoubleFollowMode("SR", "50");
    if(alignment == RIGHT_OF_LINE)
        lifo.setDoubleFollowMode("50", "SL");

    lifo.distance(robot.cmToTacho(30), startPhaseDist, NONE);
    lifo.distance(robot.cmToTacho(45), totalDistance - startPhaseDist - endPhaseDist - slowDist, NONE);
    t.reset();
    robot.resetPosition();
    lifo.distance(robot.cmToTacho(30), endPhaseDist, NONE);
    speed = robot.getPosition() / t.secElapsed();

    setLifoSlow();
    if(alignment == LEFT_OF_LINE)
        lifo.setDoubleFollowMode("SR", "50");
    if(alignment == RIGHT_OF_LINE)
        lifo.setDoubleFollowMode("50", "SL");
    lifo.setAccelParams(150, speed, 20);
    lifo.distance(20, slowDist, (detectLine != NO_DETECT) ? NONE : stopMode);
    lifo.setAccelParams(150, 20, 20);
    if(detectLine == NORMAL) 
    {
        lifo.lines(20, 1, stopMode);
    }
    else if(detectLine == COLORED)
    {
        lifo.setSensorMode(WHITE_RGB);
        lifo.unlimited(20, true);
        bool isDone = false;
        colorspaceRGB lCur, lPrev, rCur, rPrev;
        timer conditionTimer;
        lCur = lPrev = leftSensor.getRGB();
        rCur = rPrev = rightSensor.getRGB();
        while(!isDone)
        {
            lifo.unlimited(20);
            if(conditionTimer.secElapsed() > 0.01)
            {
                lCur = leftSensor.getRGB();
                rCur = rightSensor.getRGB();

                isDone = abs((lCur.red + rCur.red) - (lPrev.red + rPrev.red)) > 30
                    || abs((lCur.green + rCur.green) - (lPrev.green + rPrev.green)) > 30
                    || abs((lCur.blue + rCur.blue) - (lPrev.blue + rPrev.blue)) > 30;

                lPrev = lCur;
                rPrev = rCur;
                conditionTimer.reset();
            }
        }
        leftSensor.getReflected();
        rightSensor.getReflected();
        lifo.stop(stopMode);
    } 
    else if(detectLine == SPECIAL_REF)
    {
        lifo.unlimited(20, true);
        bool isDone = false;
        int lCur, lPrev, rCur, rPrev;
        timer conditionTimer;
        lCur = lPrev = leftSensor.getReflected();
        rCur = rPrev = rightSensor.getReflected();
        while(!isDone)
        {
            lifo.unlimited(20);
            if(conditionTimer.secElapsed() > 0.01)
            {
                lCur = leftSensor.getReflected();
                rCur = rightSensor.getReflected();

                isDone = abs((lCur + rCur) - (lPrev + rPrev)) > 10;

                lPrev = lCur;
                rPrev = rCur;
                conditionTimer.reset();
            }
        }
        lifo.stop(stopMode);
    }
}

void switchLifoRobotPosition(double speed, lifoRobotPosition startAlignment, lifoRobotPosition endAlignment)
{
    if(startAlignment == CENTERED && endAlignment != CENTERED)
    {
        robot.setMode(CONTROLLED);
        robot.setLinearAccelParams(100, speed, speed);
        robot.arc(speed, 10, (endAlignment == LEFT_OF_LINE) ? -20 : 20, NONE);
    }
    if(startAlignment == RIGHT_OF_LINE && endAlignment == CENTERED)
    {
        robot.setMode(CONTROLLED);
        robot.setLinearAccelParams(100, speed, speed);
        robot.arc(speed, 10, -20, NONE);
    }
    if(startAlignment == LEFT_OF_LINE && endAlignment == CENTERED)
    {
        robot.setMode(CONTROLLED);
        robot.setLinearAccelParams(100, speed, speed);
        robot.arc(speed, 10, 20, NONE);
    }
    currentAlignment = endAlignment;
}
