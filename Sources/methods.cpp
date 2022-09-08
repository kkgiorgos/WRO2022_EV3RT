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

    //printf("Reset Color of Line Followed\n");
}

void setLifoSlow()
{
    lifo.setDoubleFollowMode("SL", "SR");
    lifo.setPIDparams(slowKP, slowKI, slowKD, PIDspeed);
    lifo.initializeMotionMode(CONTROLLED);
    lifo.setAlignMode(true);
}

void setLifoLeft()
{
    lifo.setDoubleFollowMode("SL", "62");
    lifo.setPIDparams(KP*1.78, KI*1.78, KD*2, PIDspeed);
}

void setLifoLeftExtreme()
{
    lifo.setDoubleFollowMode("SL", "62");
    lifo.setPIDparams(KP*3, KI*1.5, KD*5, PIDspeed);   
}

void setLifoRight()
{
    lifo.setDoubleFollowMode("66", "SR");
    lifo.setPIDparams(KP*1.75, KI*1.75, KD*2, PIDspeed);
}

void setLifoRightExtreme()
{
    lifo.setDoubleFollowMode("66", "SR");
    lifo.setPIDparams(KP*3, KI*1.5, KD*5, PIDspeed);
}

void executeLifoLeftUnlim(int velocity)
{
    int refRight = rightSensor.getReflected();
    if(refRight < 15 || refRight > 80)
        lifo.setDoubleFollowMode("N", "N");
    else
        lifo.setDoubleFollowMode("SL", "62");
    lifo.unlimited(velocity);
}

void executeLifoRightUnlim(int velocity)
{
    int refLeft = leftSensor.getReflected();
    if(refLeft < 15 || refLeft > 80)
        lifo.setDoubleFollowMode("N", "N");
    else
        lifo.setDoubleFollowMode("66", "SR");
    lifo.unlimited(velocity);
}

bool detectColorLine(colorSensor &sensor, colors target)
{
    // switch(target)
    // {
    //     case RED:
    //         return sensor.getReflected() > 50;
    //     case GREEN:
    //         return sensor.getReflected() < 20;
    //     case BLUE:
    //         return sensor.getReflected() < 20;
    //     case YELLOW:
    //         return sensor.getReflected() > 80;
    // }
    return abs(sensor.getReflected() - 33) > 5;
}

bool detectWhiteRoomBed(colorSensor &sensor)
{
    return sensor.getColor() == WHITE;
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

void reverse(bool stop, bool alignEnd)
{
    robot.setMode(REGULATED);
    if(alignEnd)
    {
        robot.arc(700, 180, 0.2, NONE);
        align(0.2, stop);
    }
    else
        robot.arc(700, 180, 0.2, BRAKE);
}

void leftTurn(bool stop, bool alignEnd)
{
    if(!alignEnd)
    {
        double angle;
        
        angle = -90;

        robot.setMode(CONTROLLED);
        robot.setAngularAccelParams(1000, 0, 50);
        robot.turn(500, angle, BRAKE);
    }
    else
    {
        double center;
        double angle;

        center = -4;
        angle = 75;

        robot.setMode(REGULATED);
        robot.arc(1000, angle, center, NONE);
        align(0.2, stop);
    }
}

void rightTurn(bool stop, bool alignEnd)
{
    if(!alignEnd)
    {
        double angle;

        angle = 95;

        robot.setMode(CONTROLLED);
        robot.setAngularAccelParams(1000, 0, 50);
        robot.turn(500, angle, BRAKE);
    }
    else
    {
        double center;
        double angle;

        center = 4;
        angle = 75;

        robot.setMode(REGULATED);
        robot.arc(1000, angle, center, NONE);
        align(0.2, stop);
    }
}

void lifo1LineDist(double distance)
{
    lifo.setAccelParams(250, 5, 50);
    lifo.distance(50, distance, NONE);
    lifo.setAccelParams(600, 50, 50);
    lifo.lines(50, 1, COAST);
}

void lifo1WhiteLineLeftSlow(double startVelocity, double distance, double slowVelocity, breakMode stopMode)
{
    lifo.initializeMotionMode(CONTROLLED);
    lifo.setDoubleFollowMode("SL", "62");
    lifo.setAlignMode(true);
    lifo.setPIDparams(slowKP*1.78, slowKI*1.78, slowKD*2, 1);
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
    lifo.setDoubleFollowMode("66", "SR");
    lifo.setAlignMode(true);
    lifo.setPIDparams(slowKP*1.75, slowKI*1.75, slowKD*2, 1);
    lifo.setAccelParams(200, startVelocity, slowVelocity);
    lifo.distance(startVelocity, distance, NONE);
    lifo.unlimited(slowVelocity, true);
    while(leftSensor.getReflected() < 60)
    {
        executeLifoRightUnlim(slowVelocity);
    }
    robot.stop(stopMode);
}


void openGrabber()
{
    grabber.moveUntilStalled(-300);
}
void pickBlock()
{
    if(rampQueue.size() < 2)
    {
        grabber.moveDegrees(250, 110, breakMode::NONE);
        grabber.moveDegrees(70, 40, breakMode::BRAKE);  
    }
    else
    {
        grabber.moveDegrees(250, 100, NONE);
        grabber.moveDegrees(60, 50, BRAKE);
    }
}

void emptyRampLaundry()
{
    ramp.moveDegrees(150, 80);
    timer::secDelay(0.5);
    ramp.moveUntilStalled(-300, BRAKE);
}

void emptyRampWater()
{
    //ramp.moveDegrees(120, 80, BRAKE);
    ramp.moveDegrees(300, 120, BRAKE);
    act_tsk(CLOSE_RAMP_TASK);
    //t.secDelay(0.2);
    //ramp.moveUntilStalled(-300, BRAKE);
}

colors scanLaundryBlock(colorSensor &scanner)
{
    scanner.setNormalisation(false);
    colorspaceHSV hsv = scanner.getHSV();
    
    if(hsv.saturation > 10)
    {
        if(hsv.value < 20)
            return BLACK;
        if(hsv.hue < 20)
            return RED;
        else
            return YELLOW;
    }

    return WHITE;
}

colors scanCodeBlock(colorSensor &scanner)
{
    scanner.setNormalisation(true);
    colorspaceHSV hsv = scanner.getHSV();

    if(hsv.value > 60 && hsv.saturation < 20)
    {
        return WHITE;
    }
    else
    {
        if(hsv.hue > 110 && hsv.hue < 170 && hsv.saturation > 70)
        {
            return GREEN;
        }
        else
        {
            return BLACK;
        }
    }
}
