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

void room::scanTask()
{
    DEBUGPRINT("\nScanning %s room: ", name);
    currentState = SCANNING;

    //Scan code

    //Set task
    task = WATER;

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

void room::pickLaundry()
{
    DEBUGPRINT("Picking laundry from %s room: ", name);
    currentState = PICKING_LAUNDRY;

    //Pick Laundry Code

    //Set laundry color and add the item to the ramp queue
    laundry = RED;

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
    scanTask();
    pickLaundry();

    if(task == WATER)
        leaveWater();
    else
    {
        pickBall();
        leaveBall();
    }

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

void turnToBasket(baskets current, baskets target)
{
    //TODO
}


//Actual tasks start here

void startProcedure()
{
    DEBUGPRINT("\nStarting movement!!!\n");

    //Get out of the start position
    currentDirection = NORTH;

}

void pickWater()
{
    DEBUGPRINT("\nPicking water bottles.\n");

    //Pick First Bottle
    DEBUGPRINT("First bottle of water has been loaded.\n");
    rampQueue.push(BOTTLE);

    //Pick Second Bottle
    DEBUGPRINT("Second bottle of water has been loaded.\n");
    rampQueue.push(BOTTLE);
}


void scanLaundryBaskets()
{
    DEBUGPRINT("\nScanning laundry baskets.\n");

    //180 turn for scanning
    //Scan first basket (left most)
    DEBUGPRINT("First laundry basket was scanned and it has the color: ");
    laundryBaskets[BASKET_LEFT] = RED;
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

    //Scan second basket (middle one)
    DEBUGPRINT("Second laundry basket was scanned and it has the color: ");
    laundryBaskets[BASKET_MIDDLE] = BLACK;
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

    //Scan third basket (last one)
    DEBUGPRINT("Last laundry basket was scanned and it has the color: ");
    laundryBaskets[BASKET_RIGHT] = YELLOW;
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
    
    //Turn to the closest basket based on where scanning ends.
    baskets currentBasket = BASKET_RIGHT;
    baskets targetBasket;

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
    }
    
    //Turn to the middle to leave and fix currentOrientation
    turnToBasket(currentBasket, BASKET_MIDDLE);
    DEBUGPRINT("Finished leaving the laundry.\n");
}

void finishProcedure()
{
    DEBUGPRINT("\nEnding movement :(\n");

    //Getting inside finishing square
}
