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
}

void setLifoLeftExtreme(bool slow)
{
    lifo.setDoubleFollowMode("SL", "70");
    if(slow)
        lifo.setPIDparams(slowKP*3, slowKI*1.5, slowKD*5, PIDspeed);   
    else
        lifo.setPIDparams(KP*3, KI*1.5, KD*5, PIDspeed);   
}

void setLifoRight(bool slow)
{
    lifo.setDoubleFollowMode("70", "SR");
    if(slow)
        lifo.setPIDparams(slowKP*1.7, slowKI*1.7, slowKD*2, PIDspeed);
    else
        lifo.setPIDparams(KP*1.7, KI*1.7, KD*2, PIDspeed);
}

void setLifoRightExtreme(bool slow)
{
    lifo.setDoubleFollowMode("70", "SR");
    if(slow)
        lifo.setPIDparams(slowKP*3, slowKI*1.5, slowKD*5, PIDspeed);   
    else
        lifo.setPIDparams(KP*3, KI*1.5, KD*5, PIDspeed);   
}

void executeLifoLeftUnlim(int velocity)
{
    int refRight = rightSensor.getReflected();
    if(refRight < 15 || refRight > 80)
        lifo.setDoubleFollowMode("N", "N");
    else
        lifo.setDoubleFollowMode("SL", "70");
    lifo.unlimited(velocity);
}

void executeLifoRightUnlim(int velocity)
{
    int refLeft = leftSensor.getReflected();
    if(refLeft < 15 || refLeft > 80)
        lifo.setDoubleFollowMode("N", "N");
    else
        lifo.setDoubleFollowMode("70", "SR");
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
    // return sensor.getColor() == WHITE;
    colorspaceRGB rgb = sensor.getRGB();
    return rgb.red > 200 && rgb.green > 200 && rgb.blue > 200;
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


void openGrabber()
{
    grabber.setStallTolerance(150, 10, 0.1);
    grabber.setSpeedLimiter(false);
    grabber.setMode(REGULATED);
    grabber.moveDegrees(-1400, 300, NONE);
    grabber.moveUntilStalled(-500, BRAKE);
}

void openGrabberAsync()
{
    act_tsk(OPEN_GRABBER_TASK);
    while(!grabberUsed) tslp_tsk(10);
}

void pickBlock()
{
    int tacho;
    int startSpeed = 1400;
    int endSpeed = 700;
    int distance = 250;
    int decelDistance = 100;
    int speed;
    grabber.moveUnlimited(startSpeed, true);
    while(tacho = grabber.getTachoCount() < (distance - decelDistance))
        grabber.moveUnlimited(startSpeed);
    grabber.resetTachoCount();
    speed = startSpeed;
    while(tacho = grabber.getTachoCount() < decelDistance)
    {
        speed = (decelDistance - tacho) / decelDistance * (startSpeed - endSpeed) + endSpeed;
        grabber.moveUnlimited(speed);
        if(grabber.isStalled(speed))
            break;
    }
    grabber.moveUntilStalled(endSpeed, BRAKE);
}

void pickBlockStage1()
{
    grabber.moveDegrees(450, 90, NONE);
    grabber.moveDegrees(60, 30, NONE);
    grabberUsed = true;
    grabber.moveUntilStalled(60, BRAKE_COAST, 0.3);
    act_tsk(PICK_BLOCK_TASK);
}
void pickBlockStage2()
{
    grabberUsed = false;
    ev3_speaker_play_tone(300, 10);
} 

void openGrabberAndPickBlock()
{
    startPicking = false;
    grabber.setStallTolerance(150, 10, 0.1);
    grabber.setSpeedLimiter(false);
    grabber.setMode(REGULATED);
    grabber.moveDegrees(-1400, 300, NONE);
    grabber.moveUntilStalled(-500, BRAKE);
    while(!startPicking) tslp_tsk(10);

    int tacho;
    int startSpeed = 1400;
    int endSpeed = 700;
    int distance = 300;
    int decelDistance = 100;
    grabber.moveUnlimited(startSpeed, true);
    while(tacho = grabber.getTachoCount() < (distance - decelDistance))
        grabber.moveUnlimited(startSpeed);
    grabber.resetTachoCount();
    while(tacho = grabber.getTachoCount() < decelDistance)
        grabber.moveUnlimited((decelDistance - tacho) / decelDistance * (startSpeed - endSpeed) + endSpeed);
    grabber.moveUntilStalled(endSpeed, BRAKE);
}

void emptyRampLaundry()
{
    ramp.setMode(REGULATED);
    ramp.moveUnlimited(300, true);
    tslp_tsk(50);
    while(abs(ramp.getCurrentSpeed()) > 50)
    {
        ramp.moveUnlimited(300);
        tslp_tsk(1);
    }
    ramp.stop(BRAKE);
    tslp_tsk(100);
    ramp.moveUnlimited(-800, true);
    tslp_tsk(50);
    while(abs(ramp.getCurrentSpeed()) > 50)
    {
        ramp.moveUnlimited(-800);   
        tslp_tsk(1);
    }
    ramp.stop(BRAKE);
}

void emptyRampWater()
{
    //ramp.moveDegrees(120, 80, BRAKE);
    ramp.moveDegrees(300, 120, BRAKE);
    act_tsk(CLOSE_RAMP_TASK);
    //t.secDelay(0.2);
    //ramp.moveUntilStalled(-300, BRAKE);
}

void emptyRampWaterStage1(bool wait)
{
    // ramp.moveDegrees(700, 70, BRAKE, wait);
    ramp.moveDegrees(800, 200, BRAKE, wait);
}
void emptyRampWaterStage2()
{
    // ramp.moveUnlimited(300, true);
    // tslp_tsk(150);
    // while(abs(ramp.getCurrentSpeed()) > 50)
    // {
    //     ramp.moveUnlimited(300);
    //     tslp_tsk(1);
    // }
    // ramp.stop(BRAKE);

    ramp.moveUnlimited(500, true);
    tslp_tsk(50);
    while(abs(ramp.getCurrentSpeed()) > 200)
    {
        ramp.moveUnlimited(500);
        tslp_tsk(1);
    }
    ramp.stop(BRAKE_COAST);   
}

colors scanLaundryBlock(colorSensor &scanner)
{
    scanner.setNormalisation(false);
    colorspaceRGB rgb = scanner.getRGB();

    if(rgb.red >= 3 * rgb.green) return RED;
    if(rgb.red > 2 * rgb.blue && rgb.green > 2 * rgb.blue) return YELLOW;
    if(rgb.red > 1 && rgb.green > 1 && rgb.blue > 1 && rgb.white < 100) return BLACK;

    // colorspaceHSV hsv = scanner.getHSV();
    
    // if(hsv.saturation > 10)
    // {
    //     if(hsv.value < 20)
    //         return BLACK;
    //     if(hsv.hue < 20)
    //         return RED;
    //     else
    //         return YELLOW;
    // }

    return WHITE;
}

colors scanCodeBlock(colorSensor &scanner)
{
    scanner.setNormalisation(false);
    colorspaceRGB rgb = scanner.getRGB();
    colorspaceHSV hsv = scanner.getHSV();

    if(rgb.white <= 3) return BLACK;
    if(rgb.green >= rgb.red + rgb.blue) return GREEN;
    else return WHITE;

    // if(hsv.value > 60 && hsv.saturation < 20)
    // {
    //     return WHITE;
    // }
    // else
    // {
    //     if(hsv.hue > 110 && hsv.hue < 170 && hsv.saturation > 70)
    //     {
    //         return GREEN;
    //     }
    //     else
    //     {
    //         return BLACK;
    //     }
    // }
}

colors scanLaundryBasket(colorSensor &scanner)
{
    scanner.setNormalisation(false);
    colorspaceRGB rgb = scanner.getRGB();

    if(rgb.green > 3) return YELLOW;
    if(rgb.red > 3) return RED;
    return BLACK;
}
