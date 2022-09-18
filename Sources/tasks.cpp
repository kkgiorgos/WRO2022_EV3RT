#include "tasks.h"
#include "methods.h"
#include "ev3ys.h"
#include "globalRobot.h"

using namespace ev3ys;
using namespace ev3cxx;
using namespace std;

//Helper functions

//room class methods
room::room(colors col)
{
    currentState = WAITING;
    doLaundry = false;
    color = col;
    switch(color)
    {
        case RED:
            position = RR;
            roomOrientation = RED_BLUE;
            sprintf(name, "red");
            break;
        case BLUE:
            position = BR;
            roomOrientation = RED_BLUE;
            sprintf(name, "blue");
            break;
        case GREEN:
            position = GR;
            roomOrientation = GREEN_YELLOW;
            sprintf(name, "green");
            break;
        case YELLOW:
            position = YR;
            roomOrientation = GREEN_YELLOW;
            sprintf(name, "yellow");
            break;
    }
}

void room::report()
{
    DEBUGPRINT("\nReporting %s room's status:\n", name);
    DEBUGPRINT("Currently ");
    switch (currentState)
    {
        case WAITING:
            DEBUGPRINT("the room has not been visited.\n");
            break;
        case SCANNING:
            DEBUGPRINT("the room is being scanned and the task is");
            switch(task)
            {
                case WATER:
                    DEBUGPRINT(" to depose water.\n");
                    break;
                case BALL:
                    DEBUGPRINT(" to play a game.\n");
                    break;
            }
            break;
        case PICKING_LAUNDRY:
            switch(laundry)
            {
                case RED:
                    DEBUGPRINT("red ");
                    break;
                case BLACK:
                    DEBUGPRINT("black ");
                    break;
                case YELLOW:
                    DEBUGPRINT("yellow ");
                    break;
            }
            DEBUGPRINT("laundry is being picked.\n");
            break;
        case LEAVING_WATER:
            DEBUGPRINT("water is being deposited.\n");
            break;
        case PLAYING_BALL:
            DEBUGPRINT("a game is being played.\n");
            break;
        case COMPLETE:
            DEBUGPRINT("all tasks are complete.\n");
            break;
    }
}

matPos room::getPosition()
{
    return position;
}

void room::setTask(colors code)
{
    DEBUGPRINT("\nScanning %s room: ", name);
    currentState = SCANNING;

    //Set task
    task = (code == WHITE) ? WATER : BALL;

    DEBUGPRINT("task is ");
    if(task == WATER)
        DEBUGPRINT("to dispose water.\n");
    else if(task == BALL)
        DEBUGPRINT("to play a game.\n");
}

tasks room::getTask()
{
    return task;
}

void room::scanLaundry()
{
    DEBUGPRINT("Scanning laundry from %s room: ", name);
    //Scan laundry color and add the item to the ramp queue

    correctionOnTheMove();
    robot.setMode(CONTROLLED);
    robot.setLinearAccelParams(150, 20, 35);
    robot.straight(35, 7, NONE);
    robot.setLinearAccelParams(150, 35, 35);
    robot.straightUnlim(35, true);    
    laundry = WHITE;
    map<colors, int> appearances;
    colors current;
    bool notWhite = false;
    while(!detectWhiteRoomBed(roomOrientation == RED_BLUE ? leftSensor : rightSensor))
    {
        if((current = scanLaundryBlock(roomOrientation == RED_BLUE ? leftScanner : rightScanner)) != WHITE)
        {
            appearances[current]++;
            notWhite = true;
        }
        robot.straightUnlim(35);
    }
    if(notWhite)
    {
        int maxCount = 0;
        for(auto x: appearances)
        {
            if(x.second > maxCount)
            {
                maxCount = x.second;
                laundry = x.first;    
            }
        }
    }
    robot.setLinearAccelParams(150, 35, 10);
    robot.straight(20, (color == RED ? 6 : 6.6), BRAKE); //ADD 0.6cm if room is NOT red

    //Set if there is laundry to be done
    doLaundry = laundry != WHITE; //WHITE signifies no laundry
    if(doLaundry)
    {
        DEBUGPRINT("laundry color is ");
        switch(laundry)
        {
            case RED:
                DEBUGPRINT("red.\n");
                rampQueue.push(LAUNDRY_RED);
                break;
            case BLACK:
                DEBUGPRINT("black.\n");
                rampQueue.push(LAUNDRY_BLACK);
                break;
            case YELLOW:
                DEBUGPRINT("yellow.\n");
                rampQueue.push(LAUNDRY_YELLOW);
                break;
        }
    }
    else
    {
        DEBUGPRINT("no available laundry to be done!\n");
    }
}

/**
 * @brief 
 * 
 * @param stage Stage 1: "OPEN" grabber, Stage 2: "LOAD" laundry block on the ramp
 */
void room::pickLaundry(int stage)
{
    if(doLaundry)
    {
        //Pick Laundry Code
        if(stage == 1)
        {
            DEBUGPRINT("Picking laundry from %s room: ", name);
            currentState = PICKING_LAUNDRY;
            act_tsk(OPEN_GRABBER_TASK);
        }
        else
        {
            if(task == WATER)
                act_tsk(PICK_BLOCK_TASK);   
            else //BALL (we have to open grabber immediately after picking laundry)
                act_tsk(WATER_GRABBER_TASK);   
        }

        tslp_tsk(1);
    }
}

colors room::getLaundryColor()
{
    return laundry;
}

/**
 * @brief 
 * 
 * @param stage Stage 1: "LOAD" water bottle to the mechanism, Stage 2: "DROP" bottle on the table, Stage 3: "CLOSE" the mechanism  
 */
void room::leaveWater(int stage)
{
    //Leave Water Code
    if(stage == 1)
    {
        DEBUGPRINT("Leaving water at the %s room.\n", name);
        currentState = LEAVING_WATER;
        emptyRampWaterStage1(false);
    }
    else if(stage == 2)
    {
        emptyRampWaterStage2();
        rampQueue.pop();
    }
    else
    {
        act_tsk(CLOSE_RAMP_TASK);
        tslp_tsk(1);
    }
}

/**
 * @brief 
 * 
 * @param stage Stage 1 only without laundry "OPENS" grabber, Stage 2 "LOADS" ball to designated location
 */
void room::pickBall(int stage)
{
    //Pick Ball Code
    if(stage == 1)
    {
        act_tsk(OPEN_GRABBER_TASK);
        tslp_tsk(1);
    }
    else
    {
        DEBUGPRINT("Picking the ball at the %s room.\n", name);
        currentState = PLAYING_BALL;
        grabber.moveDegrees(600, 210, NONE, true);
        grabber.moveDegrees(400, 100, BRAKE, false);
    }
}

void room::leaveBall()
{
    DEBUGPRINT("Leave the ball at the %s room.\n", name);
    currentState = PLAYING_BALL;

    //Leave Ball Code
    act_tsk(PICK_BLOCK_TASK);
    tslp_tsk(1);
}



void room::taskWater()
{
    if(roomOrientation == GREEN_YELLOW)
    {
        correctionBeforeMovement();
        robot.setMode(REGULATED);
        robot.arc(20, -47, -8.5, NONE);    

        leaveWater(1);
        robot.arc(30, -45, -3, BRAKE);
        leaveWater(2);

        correctionBeforeMovement();
        robot.setMode(CONTROLLED);
        robot.setLinearAccelParams(150, 10, 20);
        robot.straight(35, 4, NONE);    

        robot.setMode(REGULATED);
        robot.arc(30, 45, 3, NONE);
        robot.arc(30, 45, 4, NONE);
        leaveWater(3);
        robot.arcUnlim(30, 4, FORWARD, true);
        while(rightSensor.getReflected() < 60)
            robot.arcUnlim(30, 4, FORWARD, false);
    }
    else //RED_BLUE
    {
        correctionBeforeMovement();
        robot.setMode(REGULATED);
        robot.arc(20, -47, 8.5, NONE);    

        leaveWater(1);
        robot.arc(30, -45, 4, BRAKE);
        leaveWater(2);

        correctionBeforeMovement();
        robot.setMode(CONTROLLED);
        robot.setLinearAccelParams(150, 10, 20);
        robot.straight(35, 3, NONE);    

        robot.setMode(REGULATED);
        robot.arc(30, 45, -3, NONE);
        robot.arc(30, 45, -4, NONE);
        leaveWater(3);
        robot.arcUnlim(30, -4, FORWARD, true);
        while(leftSensor.getReflected() < 60)
            robot.arcUnlim(30, -4, FORWARD, false);
    }
}

void room::taskWaterLaundry()
{
    if(roomOrientation == GREEN_YELLOW)
    {
        pickLaundry(1);
        correctionBeforeMovement();
        robot.setMode(REGULATED);
        robot.arc(20, -47, -8.5, NONE);    

        leaveWater(1);
        robot.arc(30, -45, -3, BRAKE);
        leaveWater(2);

        correctionBeforeMovement();
        robot.setMode(CONTROLLED);
        robot.setLinearAccelParams(150, 10, 35);
        robot.straight(35, 7, NONE);    
        leaveWater(3);
        pickLaundry(2);
        robot.setLinearAccelParams(150, 35, 10);
        robot.straight(30, 9, BRAKE);

        robot.setLinearAccelParams(150, -10, -10);
        robot.straight(35, -11, BRAKE);

        correctionBeforeMovement();
        robot.setMode(REGULATED);
        robot.arc(30, 92, 3.5, NONE);
        robot.arcUnlim(30, 3.5, FORWARD, true);
        while(rightSensor.getReflected() < 60)
            robot.arcUnlim(30, 3.5, FORWARD, false);
    }
    else //RED_BLUE
    {
        pickLaundry(1);
        correctionBeforeMovement();
        robot.setMode(REGULATED);
        robot.arc(20, -47, 8.5, NONE);    

        leaveWater(1);
        robot.arc(30, -45, 3, BRAKE);
        leaveWater(2);

        correctionBeforeMovement();
        robot.setMode(CONTROLLED);
        robot.setLinearAccelParams(150, 10, 35);
        robot.straight(35, 7, NONE);    
        leaveWater(3);
        pickLaundry(2);
        robot.setLinearAccelParams(150, 35, 10);
        robot.straight(30, 9, BRAKE);

        robot.setLinearAccelParams(150, -10, -10);
        robot.straight(35, -11, BRAKE);

        correctionBeforeMovement();
        robot.setMode(REGULATED);
        robot.arc(30, 92, -3.5, NONE);
        robot.arcUnlim(30, -3.5, FORWARD, true);
        while(leftSensor.getReflected() < 60)
            robot.arcUnlim(30, -3.5, FORWARD, false);
    }
}

void room::taskBall()
{
    if(roomOrientation == GREEN_YELLOW)
    {
        pickBall(1);
        robot.setMode(REGULATED);
        robot.arc(25, 50, 5, BRAKE);    
        while(grabberUsed)
            tslp_tsk(10); 
        robot.setMode(CONTROLLED);
        robot.setLinearAccelParams(150, 20, 20);
        robot.straight(25, 5, BRAKE);
        pickBall(2);
        robot.setMode(REGULATED);
        robot.arc(30, -115, 2, BRAKE);
        robot.setLinearAccelParams(150, 15, 20);
        robot.straight(45, 15.5, NONE);
        leaveBall();
        robot.setLinearAccelParams(150, 20, 15);
        robot.straight(20, 1, BRAKE);
        robot.setLinearAccelParams(150, -15, -15);
        robot.straight(45, -9.5, BRAKE);
        robot.setAngularAccelParams(600, -200, -200);
        robot.turn(300, -115, BRAKE);
        robot.setLinearAccelParams(150, 15, 35);
        robot.straight(40, 17, NONE);
    }
    else    //RED_BLUE
    {
        pickBall(1);
        robot.setMode(REGULATED);
        robot.arc(25, 50, -5, BRAKE);    
        while(grabberUsed)
            tslp_tsk(10); 
        robot.setMode(CONTROLLED);
        robot.setLinearAccelParams(150, 20, 20);
        robot.straight(25, 5, BRAKE);
        pickBall(2);
        robot.setMode(REGULATED);
        robot.arc(30, -115, -2, BRAKE);
        robot.setLinearAccelParams(150, 15, 20);
        robot.straight(45, 15, NONE);
        leaveBall();
        robot.setLinearAccelParams(150, 20, 15);
        robot.straight(20, 1, BRAKE);
        robot.setLinearAccelParams(150, -15, -15);
        robot.straight(45, -8, BRAKE);
        robot.setAngularAccelParams(600, 200, 200);
        robot.turn(300, 128, BRAKE);
        robot.setLinearAccelParams(150, 15, 35);
        robot.straight(40, 17, NONE);   
    }
}

void room::taskBallLaundry()
{
    if(roomOrientation == GREEN_YELLOW)
    {
        pickLaundry(1);
        robot.setMode(REGULATED);
        robot.arc(25, -45, -8.5, NONE);    

        robot.arc(30, -45, -3, BRAKE);

        robot.setMode(CONTROLLED);
        robot.setLinearAccelParams(150, 15, 35);
        robot.straight(35, 5, NONE);    
        pickLaundry(2);
        robot.setLinearAccelParams(150, 35, 20);
        robot.straight(30, 12, NONE);

        robot.setMode(REGULATED);
        robot.arc(30, 91, -3.5);
        while(grabberUsed)
            tslp_tsk(10); 

        robot.setMode(CONTROLLED);
        robot.setLinearAccelParams(150, 20, 20);
        robot.straight(25, 7, BRAKE);
        pickBall(2);
        robot.setMode(REGULATED);
        robot.arc(30, 82, -5, NONE);

        robot.setMode(CONTROLLED);
        robot.setLinearAccelParams(150, 15, 20);
        robot.straight(45, 17.5, NONE);
        leaveBall();
        robot.setLinearAccelParams(150, 20, 15);
        robot.straight(20, 1, BRAKE);
        robot.setLinearAccelParams(150, -15, -15);
        robot.straight(45, -8, BRAKE);
        robot.setAngularAccelParams(600, -200, -200);
        robot.turn(300, -103, BRAKE);
        robot.setLinearAccelParams(150, 15, 35);
        robot.straight(40, 19.5, NONE);
    }
    else    //RED_BLUE
    {
        pickLaundry(1);
        robot.setMode(REGULATED);
        robot.arc(25, -45, 8.5, NONE);    

        robot.arc(30, -45, 3, BRAKE);

        robot.setMode(CONTROLLED);
        robot.setLinearAccelParams(150, 15, 35);
        robot.straight(35, 5, NONE);    
        pickLaundry(2);
        robot.setLinearAccelParams(150, 35, 20);
        robot.straight(30, 12, NONE);

        robot.setMode(REGULATED);
        robot.arc(30, 95, 3.5);
        while(grabberUsed)
            tslp_tsk(10); 

        robot.setMode(CONTROLLED);
        robot.setLinearAccelParams(150, 20, 20);
        robot.straight(25, 7, BRAKE);
        pickBall(2);
        robot.setMode(REGULATED);
        robot.arc(30, 82, 5, NONE);

        robot.setMode(CONTROLLED);
        robot.setLinearAccelParams(150, 15, 20);
        robot.straight(45, 17.5, NONE);
        leaveBall();
        robot.setLinearAccelParams(150, 20, 15);
        robot.straight(20, 1, BRAKE);
        robot.setLinearAccelParams(150, -15, -15);
        robot.straight(45, -8, BRAKE);
        robot.setAngularAccelParams(600, 200, 200);
        robot.turn(300, 110, BRAKE);
        robot.setLinearAccelParams(150, 15, 35);
        robot.straight(40, 19.5, NONE);
    }
}

void room::enterRoom()
{
    //Setup code for everything to happen inside the room
    //Robot assumes position either to pickLaundry or to move further inside
    
    scanLaundry();
}

void room::executeTask()
{
    if(task == WATER)
    {
        if(doLaundry)
            taskWaterLaundry();
        else //NO LAUNDRY
            taskWater();
    }
    else //BALL
    {
        if(doLaundry)
            taskBallLaundry();
        else //NO LAUNDRY
            taskBall();
    }
}

void room::exitRoom()
{
    DEBUGPRINT("Exiting the %s room.\n", name);
    currentState = COMPLETE;

    //Exiting code
    if(!(!doLaundry && task == WATER))
        grabber.moveDegrees(-300, 110, BRAKE, false);
}

void room::executeAllActions()
{
    enterRoom();

    executeTask();

    exitRoom();
}

//Helper functions

void printRampQueue()
{
    queue<items, list<items>> temp = rampQueue;
    int size = temp.size();
    DEBUGPRINT("\nCurrent State of the Ramp Queue: [front, ");
    for(int i = 0; i < size; i++)
    {
        switch(temp.front())
        {
            case BOTTLE:
                DEBUGPRINT("BOTTLE, ");
                break;
            case LAUNDRY_BLACK:
                DEBUGPRINT("LAUNDRY_BLACK, ");
                break;
            case LAUNDRY_RED:
                DEBUGPRINT("LAUNDRY_RED, ");
                break;
            case LAUNDRY_YELLOW:
                DEBUGPRINT("LAUNDRY_YELLOW, ");
                break;
        }
        temp.pop();
    }
    DEBUGPRINT("back]\n");
}

tasks findTask(colors color)
{
    if(color == WHITE)
        return WATER;
    else if(color == GREEN)
        return BALL;
}

colors findColorOfItem(items item)
{
    switch(item)
    {
        case LAUNDRY_BLACK:
            return BLACK;
        case LAUNDRY_RED:
            return RED;
        case LAUNDRY_YELLOW:
            return YELLOW;
        default:
            return WHITE;   //Should not happen
    }
}

baskets findBasket(colors color)
{
    int result = -1;
    for(int i = 0; i < 3; i++)
    {
        if(laundryBaskets[i] == color)
            result = i;
    }
    return static_cast<baskets>(result);
}

colors clasifyBasket(colorspaceHSV hsv)
{
    if(hsv.saturation > 40)
    {
        if(hsv.hue > 4)
            return YELLOW;
        else 
            return RED;
    }
    else
        return BLACK;
}

colors findTheLastColor(colors *cols, int numOfCols)
{
    colors available[] = {RED, YELLOW, BLACK};
    bool hasAppeared[] = {false, false, false};
    colors last;
    for(int i = 0; i < numOfCols - 1; i++)
    {
        for(int j = 0; j < numOfCols; j++)
        {
            if(available[j] == cols[i])
                hasAppeared[j] = true;
        }
    }
    for(int i = 0; i < numOfCols; i++)
    {
        if(!hasAppeared[i])
            return available[i];
    }
}

void turnToBasket(baskets current, baskets target)
{
    //TODO
    int turnDifference = target - current;
    if(turnDifference != 0)
        robot.turn(300, -36.5 * turnDifference);
}


//Actual tasks start here

void startProcedure()
{
    DEBUGPRINT("\nStarting movement!!!\n");

    robot.setMode(CONTROLLED);
    robot.setAngularAccelParams(600, -200, -200);
    robot.turn(300, -45, BRAKE);

    robot.setLinearAccelParams(150, 10, 30);
    robot.straight(45, 15, NONE);

    //Get out of the start position
    currentDirection = NORTH;

}

void pickWater()
{
    DEBUGPRINT("\nPicking water bottles.\n");
    
    //Pick First Bottle
    robot.setLinearAccelParams(150, 20, 25);
    robot.straight(25, 5, COAST);
    act_tsk(WATER_GRABBER_TASK);
    tslp_tsk(1);
    robot.setLinearAccelParams(150, 25, 25);
    robot.straight(25, 2, BRAKE);
    while(grabber.getTachoCount() < 200) 
        tslp_tsk(10);
    DEBUGPRINT("First bottle of water has been loaded.\n");
    rampQueue.push(BOTTLE);

    //Pick Second Bottle
    robot.setMode(REGULATED);
    robot.arc(35, -34, -8.5);
    while(grabberUsed)
        tslp_tsk(10);

    robot.setMode(CONTROLLED);
    robot.setLinearAccelParams(150, 10, 30);
    robot.straight(30, 10, COAST);
    act_tsk(PICK_BLOCK_TASK);
    tslp_tsk(1);
    robot.setLinearAccelParams(150, 30, 30);
    robot.straight(30, 3, BRAKE);
    while(grabber.getTachoCount() < 200) 
        tslp_tsk(10);
    DEBUGPRINT("Second bottle of water has been loaded.\n");
    rampQueue.push(BOTTLE);

    robot.arc(35, 90, 3, NONE);

    robot.setMode(CONTROLLED);
    robot.setLinearAccelParams(150, 30, 30);
    robot.straight(30, 1, NONE);
    robot.straightUnlim(30, true);
    while(rightSensor.getReflected() > 80) tslp_tsk(1);
    while(leftSensor.getReflected() > 80) tslp_tsk(1);
    resetLifo();
    lifo.setPIDparams(KP*1.5, slowKI * 0.7, KD*2, 1);
    lifo.distance(robot.cmToTacho(30), 6, NONE);
    setLifoSlow();
    lifo.setAccelParams(150, 30, 30);
    lifo.distance(30, 4, NONE);
    lifo.lines(30, 1, NONE);
    grabber.stop(COAST);
}


void scanLaundryBaskets()
{
    DEBUGPRINT("\nScanning laundry baskets.\n");

    robot.setMode(CONTROLLED);
    robot.setLinearAccelParams(200, 50, 0);
    robot.setAngularAccelParams(1000, 0, 100);

    robot.straight(50, 2.5);
    timer::secDelay(0.2);
    robot.turn(200, -27);
    //Scan first basket (left most)
    colorspaceHSV hsvLeft = rightScanner.getHSV();
    robot.turn(200, 57);
    //Scan third basket (right one)
    colorspaceHSV hsvRight = leftScanner.getHSV();
    robot.turn(500, 150);
    align(0.2, true);

    //Calculating Colors
    laundryBaskets[BASKET_LEFT] = clasifyBasket(hsvLeft);
    laundryBaskets[BASKET_RIGHT] = clasifyBasket(hsvRight);

    colors temp[2];
    temp[0] = laundryBaskets[BASKET_LEFT];
    temp[1] = laundryBaskets[BASKET_RIGHT];
    laundryBaskets[BASKET_MIDDLE] = findTheLastColor(temp, 3);


    DEBUGPRINT("First laundry basket was scanned and it has the color: ");
    switch(laundryBaskets[BASKET_LEFT])
    {
        case RED:
            DEBUGPRINT("red.\n");
            break;
        case BLACK:
            DEBUGPRINT("black.\n");
            break;
        case YELLOW:
            DEBUGPRINT("yellow.\n");
            break;
    }

    DEBUGPRINT("Second laundry basket was scanned and it has the color: ");
    switch(laundryBaskets[BASKET_MIDDLE])
    {
        case RED:
            DEBUGPRINT("red.\n");
            break;
        case BLACK:
            DEBUGPRINT("black.\n");
            break;
        case YELLOW:
            DEBUGPRINT("yellow.\n");
            break;
    }
    
    DEBUGPRINT("Last laundry basket was scanned and it has the color: ");
    switch(laundryBaskets[BASKET_RIGHT])
    {
        case RED:
            DEBUGPRINT("red.\n");
            break;
        case BLACK:
            DEBUGPRINT("black.\n");
            break;
        case YELLOW:
            DEBUGPRINT("yellow.\n");
            break;
    }

    DEBUGPRINT("Finished Scanning laundry baskets.\n");
}

void leaveLaundry()
{
    DEBUGPRINT("\nLeaving Laundry.\n");

    baskets currentBasket = BASKET_MIDDLE;
    baskets targetBasket;

    robot.stop(BRAKE);
    robot.setMode(CONTROLLED);
    robot.setLinearAccelParams(100, 0, 0);
    robot.setAngularAccelParams(1000, 0, 100);

    while(!rampQueue.empty())   //Repeat for every item
    {
        //Goto the basket of the current ramp item
        targetBasket = findBasket(findColorOfItem(rampQueue.front()));
        DEBUGPRINT("Turning to ");
        switch(findColorOfItem(rampQueue.front()))
        {
            case RED:
                DEBUGPRINT("red");
                break;
            case BLACK:
                DEBUGPRINT("black");
                break;
            case YELLOW:
                DEBUGPRINT("yellow");
                break;
        }
        DEBUGPRINT(" basket located ");
        switch(targetBasket)
        {
            case BASKET_LEFT:
                DEBUGPRINT("on the left.\n");
                break;
            case BASKET_MIDDLE:
                DEBUGPRINT("in the middle.\n");
                break;
            case BASKET_RIGHT:
                DEBUGPRINT("on the right.\n");
                break;
        }
        turnToBasket(currentBasket, targetBasket);
        currentBasket = targetBasket;
        rampQueue.pop();

        //Leave the laundry
        if(currentBasket == BASKET_MIDDLE)
        {
            robot.straight(30, 4);
            emptyRampLaundry();
            robot.straight(-30, 4);
        }
        else
        {
            emptyRampLaundry();
        }
    }

    //Turn to the middle to leave and fix currentOrientation
    turnToBasket(currentBasket, BASKET_MIDDLE);
    align(0.2);
    DEBUGPRINT("Finished leaving the laundry.\n");
}

void finishProcedure()
{
    DEBUGPRINT("\nEnding movement :(\n");

    //Getting inside finishing square
    robot.setMode(CONTROLLED);
    robot.setLinearAccelParams(100, 50, 0);
    robot.straight(50, 27);
    robot.setAngularAccelParams(1000, 0, 50);
    robot.turn(500, 45);
}
