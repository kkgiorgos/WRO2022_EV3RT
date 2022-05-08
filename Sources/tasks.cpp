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
            roomOrientation = RIGHT;
            sprintf(name, "red");
            break;
        case BLUE:
            position = BR;
            roomOrientation = RIGHT;
            sprintf(name, "blue");
            break;
        case GREEN:
            position = GR;
            roomOrientation = LEFT;
            sprintf(name, "green");
            break;
        case YELLOW:
            position = YR;
            roomOrientation = LEFT;
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

    robot.setMode(CONTROLLED);
    robot.setLinearAccelParams(200, 40, 40);
    robot.straightUnlim(40, true);    
    laundry = WHITE;
    map<colors, int> appearances;
    colors current;
    bool notWhite = false;
    while((roomOrientation == RIGHT ? leftSensor : rightSensor).getReflected() < 80)
    {
        if((current = scanLaundryBlock(roomOrientation == RIGHT ? leftScanner : rightScanner)) != WHITE)
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
                laundry = x.first;    
            }
        }
    }
    robot.stop(BRAKE);

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

void room::pickLaundry()
{
    if(doLaundry)
    {
        DEBUGPRINT("Picking laundry from %s room: ", name);
        currentState = PICKING_LAUNDRY;

        //Pick Laundry Code

    }
}

colors room::getLaundryColor()
{
    return laundry;
}

void room::leaveWater()
{
    DEBUGPRINT("Leaving water at the %s room.\n", name);
    currentState = LEAVING_WATER;

    //Leave Water Code
    rampQueue.pop();

}

void room::pickBall()
{
    DEBUGPRINT("Picking the ball at the %s room.\n", name);
    currentState = PLAYING_BALL;

    //Pick Ball Code

}

void room::leaveBall()
{
    DEBUGPRINT("Leave the ball at the %s room.\n", name);
    currentState = PLAYING_BALL;

    //Leave Ball Code

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
        //Turn to correct direction
        if(roomOrientation == RIGHT)
        {
            robot.setMode(REGULATED);
            robot.arc(800, -90, 3, BRAKE);
        }
        else
        {
            //TODO
        }
        leaveWater();
        pickLaundry();
    }
    else
    {
        if(doLaundry)
        {
            if(roomOrientation == RIGHT)
            {
                robot.setMode(REGULATED);
                robot.arc(800, -90, 3, BRAKE);
            }
            else
            {
                //TODO
            }
            pickLaundry();
            //Turn to the expected BALL direction
        }
        else
        {
            //Turn to the expected BALL direction
        }
        pickBall();
        leaveBall();
    }
}

void room::exitRoom()
{
    DEBUGPRINT("Exiting the %s room.\n", name);
    currentState = COMPLETE;

    //Exiting code

    if(task == WATER)
    {
        //Exiting code after WATER task

    }
    else
    {
        //Exiting code after BALL - GAME task

    }

    //Fix currentOrientation
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
    robot.setAngularAccelParams(1000, 0, 50);
    robot.turn(500, -45);

    robot.setLinearAccelParams(200, 0, 0);
    robot.straight(50, 15);

    //Get out of the start position
    currentDirection = NORTH;

}

void pickWater()
{
    DEBUGPRINT("\nPicking water bottles.\n");

    //Pick First Bottle
    robot.setLinearAccelParams(200, 0, 0);
    robot.straight(50, 5);
    pickBlock();
    DEBUGPRINT("First bottle of water has been loaded.\n");
    rampQueue.push(BOTTLE);

    //Pick Second Bottle
    robot.setAngularAccelParams(1000, 0, 50);
    robot.arc(50, -37, -9);
    openGrabber();
    robot.straight(35, 15);
    robot.straight(-10, 1);
    pickBlock();
    DEBUGPRINT("Second bottle of water has been loaded.\n");
    rampQueue.push(BOTTLE);
}


void scanLaundryBaskets()
{
    DEBUGPRINT("\nScanning laundry baskets.\n");

    timer t;
    robot.setMode(CONTROLLED);
    robot.setLinearAccelParams(200, 50, 0);
    robot.setAngularAccelParams(1000, 0, 100);

    robot.straight(50, 2.5);
    t.secDelay(0.2);
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
