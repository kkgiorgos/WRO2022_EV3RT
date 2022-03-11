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
        int error = leftSensor.getRGB().white - rightSensor.getRGB().white;
        leftMotor.moveUnlimited(error * kp);
        rightMotor.moveUnlimited(error * kp * -1);
        tslp_tsk(1);
    }
    robot.stop(stop ? BRAKE : COAST);
    robot.setMode(prevMode);
}

void alignOnMove(int speed) //This will change the robot mode to CONTROLLED
{
    speedMode lastMode = robot.getMode();
    robot.setMode(speedMode::CONTROLLED);
    robot.tankUnlim(speed, speed, true);

    while(rightSensor.getRGB().white > 5 && leftSensor.getRGB().white > 5);
    bool isRight = rightSensor.getRGB().white < 25;

    robot.resetPosition();
    if(isRight)
        while(leftSensor.getRGB().white > 5);
    else
        while(rightSensor.getRGB().white > 5);

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

        center = -5;
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

        center = 5;
        angle = 75;

        robot.setMode(REGULATED);
        robot.arc(1000, angle, center, NONE);
        align(0.2, stop);
    }
}

void lifo1LineDist(double distance)
{
    lifo.setAccelParams(600, 0, 50);
    lifo.distance(50, distance, NONE);
    lifo.setAccelParams(600, 50, 50);
    lifo.lines(50, 1, COAST);
}

