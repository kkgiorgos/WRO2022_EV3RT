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
    printf("\nReporting %s room's status:\n", name);
    printf("Currently ");
    switch (currentState)
    {
        case WAITING:
            printf("the room has not been visited.\n");
            break;
        case SCANNING:
            printf("the room is being scanned and the task is");
            switch(task)
            {
                case WATER:
                    printf(" to depose water.\n");
                    break;
                case BALL:
                    printf(" to play a game.\n");
                    break;
            }
            break;
        case PICKING_LAUNDRY:
            switch(laundry)
            {
                case RED:
                    printf("red ");
                    break;
                case BLACK:
                    printf("black ");
                    break;
                case YELLOW:
                    printf("yellow ");
                    break;
            }
            printf("laundry is being picked.\n");
            break;
        case LEAVING_WATER:
            printf("water is being deposited.\n");
            break;
        case PLAYING_BALL:
            printf("a game is being played.\n");
            break;
        case COMPLETE:
            printf("all tasks are complete.\n");
            break;
    }
}

matPos room::getPosition()
{
    return position;
}

void room::scanTask()
{
    printf("Scanning %d room: ", name);
    currentState = SCANNING;

    //Scan code

    //Set task
    task = WATER;

    printf("task is ");
    if(task == WATER)
        printf("to dispose water.\n");
    else if(task == BALL)
        printf("to play a game.\n");
}

tasks room::getTask()
{
    return task;
}

void room::pickLaundry()
{
    printf("Picking laundry from %s room: ", name);
    currentState = PICKING_LAUNDRY;

    //Pick Laundry Code

    //Set laundry color and add the item to the ramp queue
    laundry = RED;

    printf("laundry color is ");
    switch(laundry)
    {
        case RED:
            printf("red.\n");
            //rampQueue.push(LAUNDRY_RED);
            break;
        case BLACK:
            printf("black.\n");
            //rampQueue.push(LAUNDRY_BLACK);
            break;
        case YELLOW:
            printf("yellow.\n");
            //rampQueue.push(LAUNDRY_YELLOW);
            break;
    }
}

colors room::getLaundryColor()
{
    return laundry;
}

void room::leaveWater()
{
    printf("Leaving water at the %s room.\n", name);
    currentState = LEAVING_WATER;

    //Leave Water Code

}

void room::pickBall()
{
    printf("Picking the ball at the %s room.\n", name);
    currentState = PLAYING_BALL;

    //Pick Ball Code

}

void room::leaveBall()
{
    printf("Leave the ball at the %s room.\n", name);
    currentState = PLAYING_BALL;

    //Leave Ball Code

}

void room::exitRoom()
{
    printf("Exiting the %s room.\n", name);
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

//Actual tasks start here

void startProcedure()
{
    printf("Starting movement!!!\n");

    //Get out of the start position
    //currentDirection = NORTH;

}

void pickWater()
{
    printf("Picking water bottles.\n");

    //Pick First Bottle
    printf("First bottle of water has been loaded.\n");
    //rampQueue.push(BOTTLE);

    //Pick Second Bottle
    printf("Second bottle of water has been loaded.\n");
    //rampQueue.push(BOTTLE);
}


void scanLaundryBaskets()
{
    printf("Scanning laundry baskets.\n");

    //180 turn for scanning
    //Scan first basket (left most)
    printf("First laundry basket was scanned and it has the color: ");
    laundryBaskets[0] = RED;
    switch(laundryBaskets[0])
    {
        case RED:
            printf("red.\n");
            break;
        case BLACK:
            printf("black.\n");
            break;
        case YELLOW:
            printf("yellow.\n");
            break;
    }

    //Scan second basket (middle one)
    printf("Second laundry basket was scanned and it has the color: ");
    laundryBaskets[1] = BLACK;
    switch(laundryBaskets[1])
    {
        case RED:
            printf("red.\n");
            break;
        case BLACK:
            printf("black.\n");
            break;
        case YELLOW:
            printf("yellow.\n");
            break;
    }

    //Scan third basket (last one)
    printf("Last laundry basket was scanned and it has the color: ");
    laundryBaskets[2] = YELLOW;
    switch(laundryBaskets[2])
    {
        case RED:
            printf("red.\n");
            break;
        case BLACK:
            printf("black.\n");
            break;
        case YELLOW:
            printf("yellow.\n");
            break;
    }

    printf("Finished Scanning laundry baskets.\n");
}

void leaveLaundry()
{
    printf("Leaving Laundry.\n");
    

}

void finishProcedure()
{
    printf("Ending movement :(");

    //Getting inside finishing square
}
