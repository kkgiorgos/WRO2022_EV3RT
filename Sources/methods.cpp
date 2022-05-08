#include "methods.h"
#include "ev3ys.h"
#include "globalRobot.h"
#include <cmath>

using namespace std;
using namespace ev3ys;
using namespace ev3cxx;

void resetLifo()
{
    lifo.setDoubleFollowMode("SL", "SR");
    lifo.setPIDparams(KP , KI , KD , PIDspeed);
    colorCoef = 1;

    //printf("Reset Color of Line Followed\n");
}

bool detectColorLine(colorSensor &sensor, colors target)
{
    switch(target)
    {
        case RED:
            return sensor.getReflected() > 50;
        case GREEN:
            return sensor.getReflected() < 20;
        case BLUE:
            return sensor.getReflected() < 20;
        case YELLOW:
            return sensor.getReflected() > 80;
    }
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
    double kp = 0.5 * colorCoef;
    int target = 50;
    robot.setMode(speedMode::UNREGULATED);
    while(t.secElapsed() < time)
    {
        int leftError = leftSensor.getReflected() - target;
        int rightError = rightSensor.getReflected() - target;
        robot.tankUnlim(leftError * kp, rightError * kp);
        t.secDelay(0.001);
    }
    robot.stop(stop ? BRAKE : COAST);
    robot.setMode(prevMode);
}

void alignOnMove(double speed) //This will change the robot mode to CONTROLLED
{
    speedMode lastMode = robot.getMode();
    robot.setMode(speedMode::CONTROLLED);
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
    robot.arc(speed, angle, radius, COAST);
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

        angle = 90;

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

void openGrabber()
{
    grabber.moveUntilStalled(-300);
}
void pickBlock()
{
    grabber.moveDegrees(250, 110, breakMode::NONE);
    grabber.moveDegrees(70, 40, breakMode::BRAKE);
}

void emptyRampLaundry()
{
    timer t;
    ramp.moveDegrees(150, 60, BRAKE);
    t.secDelay(0.5);
    ramp.moveUntilStalled(-300, BRAKE);
}

void emptyRampWater()
{
    timer t;
    ramp.moveDegrees(120, 80, BRAKE);
    t.secDelay(0.2);
    ramp.moveUntilStalled(-300, BRAKE);
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
