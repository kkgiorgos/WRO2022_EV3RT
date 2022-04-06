#include "methods.h"
#include "ev3ys.h"
#include "globalRobot.h"
#include <cmath>

using namespace std;
using namespace ev3ys;
using namespace ev3cxx;

void resetLifo()
{
    lifo.setPIDparams(KP , KI , KD , 1000);
    colorCoef = 1;

    printf("Reset Color of Line Followed\n");
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

void alignOnMove(int speed) //This will change the robot mode to CONTROLLED
{
    speedMode lastMode = robot.getMode();
    robot.setMode(speedMode::CONTROLLED);
    robot.tankUnlim(speed, speed, true);

    while(rightSensor.getReflected() > 5 && leftSensor.getReflected() > 5);
    bool isRight = rightSensor.getReflected() < 25;

    robot.resetPosition();
    if(isRight)
        while(leftSensor.getReflected() > 5);
    else
        while(rightSensor.getReflected() > 5);

    double length = robot.getPosition();
    double sensDiff = 1.3;
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
    lifo.setAccelParams(600, 0, 60);
    lifo.distance(60, distance, NONE);
    lifo.setAccelParams(600, 60, 60);
    lifo.lines(60, 1, COAST);
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

void emptyRamp()
{
    timer t;
    ramp.moveDegrees(150, 60, BRAKE);
    t.secDelay(0.5);
    ramp.moveUntilStalled(-300, BRAKE);
}

colors scanBlock(colorSensor &scanner)
{
    colorspaceHSV hsv = scanner.getHSV();

    if(hsv.saturation > 90)
    {
        if(hsv.hue > 110 && hsv.hue < 130)
            return GREEN;
        else if(hsv.hue >= 0 && hsv.hue < 10)
            return RED;
        else if(hsv.hue >= 10  && hsv.hue < 70)
            return YELLOW;
    }
    else
    {
        if(hsv.value < 30)
            return BLACK;
        else
            return WHITE;
    }
}
