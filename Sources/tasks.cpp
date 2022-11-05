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
            position = R;
            roomOrientation = RED_BLUE;
            sprintf(name, "red");
            break;
        case BLUE:
            position = B;
            roomOrientation = RED_BLUE;
            sprintf(name, "blue");
            break;
        case GREEN:
            position = G;
            roomOrientation = GREEN_YELLOW;
            sprintf(name, "green");
            break;
        case YELLOW:
            position = Y;
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
    task = findTask(code);

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

    //Task setup
    scanner = (roomOrientation == GREEN_YELLOW) ? &rightScanner : &leftScanner;
    lineDetector = (roomOrientation == GREEN_YELLOW) ? &rightSensor : &leftSensor;
    roomScanStage = 1;
    act_tsk(ROOM_ENTRANCE_TASK);
    tslp_tsk(1);

    //Get in the room
    robot.setMode(CONTROLLED);
    robot.setLinearAccelParams(100, 30, 30);
    robot.straightUnlim(30, true);
    while(roomScanStage != 4)
    {
        robot.straightUnlim(30);
        // tslp_tsk(1);
    }

    robot.setLinearAccelParams(100, 30, 0);
    robot.straight(30, (color == RED) ? 5.8 : 6.4, COAST);
    laundry = scannedValue;

    leftSensor.getReflected();
    rightSensor.getReflected();
    leftScanner.getReflected();
    rightScanner.getReflected();

    //Set if there is laundry to be done
    doLaundry = laundry != NO_COLOR; //WHITE signifies no laundry
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
        grabber.moveDegrees(800, 200, NONE, true);
        grabber.moveDegrees(400, 95, BRAKE, false);//110
    }
}

void room::leaveBall()
{
    DEBUGPRINT("Leave the ball at the %s room.\n", name);
    currentState = PLAYING_BALL;

    //Leave Ball Code
    grabber.setMode(REGULATED);
    grabber.moveDegrees(400, 50, COAST, true);
    act_tsk(LEAVE_BALL_TASK);
    tslp_tsk(1);
}



void room::taskWater()
{
    if(roomOrientation == GREEN_YELLOW)
    {
        leaveWater(1);
        robot.setLinearAccelParams(100, 0, -25);
        robot.arc(35, -48, -8.5, NONE);
        
        robot.setLinearAccelParams(100, -30, 0);
        robot.arc(35, -48, -4, COAST);
        leaveWater(2);

        robot.setLinearAccelParams(100, 0, 20);
        robot.straight(25, 4, NONE);

        robot.setLinearAccelParams(100, 20, 25);
        robot.arc(50, 100, 3, NONE);
        robot.setLinearAccelParams(100, 25, 25);
        robot.arcUnlim(25, 3, FORWARD, true);
        while(rightSensor.getReflected() < 80 && abs(robot.getAngle()) < 15)
            robot.arcUnlim(25, 3, FORWARD);   
        // robot.stop(COAST);

        leaveWater(3);
    }
    else //RED_BLUE
    {
        leaveWater(1);
        robot.setLinearAccelParams(100, 0, -25);
        robot.arc(35, -48, 8.5, NONE);
        
        robot.setLinearAccelParams(100, -30, 0);
        robot.arc(35, -48, 4, COAST);
        leaveWater(2);

        robot.setLinearAccelParams(100, 0, 20);
        robot.straight(25, 4, NONE);

        robot.setLinearAccelParams(100, 20, 25);
        robot.arc(50, 100, -3, NONE);
        robot.setLinearAccelParams(100, 25, 25);
        robot.arcUnlim(25, -3, FORWARD, true);
        while(leftSensor.getReflected() < 80 && abs(robot.getAngle()) < 15)
            robot.arcUnlim(25, -3, FORWARD);   
        // robot.stop(COAST);

        leaveWater(3);
    }
}

void room::taskWaterLaundry()
{
    if(roomOrientation == GREEN_YELLOW)
    {
        pickLaundry(1);
        leaveWater(1);
        robot.setLinearAccelParams(100, 0, -25);
        robot.arc(35, -48, -8.5, NONE);
        
        robot.setLinearAccelParams(100, -30, 0);
        robot.arc(35, -48, -4, COAST);
        leaveWater(2);

        robot.setLinearAccelParams(100, 0, 40);
        robot.straight(40, 7, NONE);
        leaveWater(3);
        pickLaundry(2);
        robot.setLinearAccelParams(100, 40, 0);
        robot.straight(40, 5, COAST);

        robot.setLinearAccelParams(100, 0, 0);
        robot.straight(40, -9, COAST);

        robot.setLinearAccelParams(100, 20, 25);
        robot.arc(50, 100, 3.5, NONE);
        robot.setLinearAccelParams(100, 25, 25);
        robot.arcUnlim(25, 3.5, FORWARD, true);
        while(rightSensor.getReflected() < 80 && abs(robot.getAngle()) < 15)
            robot.arcUnlim(25, 3.5, FORWARD); 
        // robot.stop(COAST);

    }
    else //RED_BLUE
    {
        pickLaundry(1);
        leaveWater(1);
        robot.setLinearAccelParams(100, 0, -25);
        robot.arc(35, -48, 8.5, NONE);
        
        robot.setLinearAccelParams(100, -30, 0);
        robot.arc(35, -48, 4, COAST);
        leaveWater(2);

        robot.setLinearAccelParams(100, 0, 40);
        robot.straight(40, 7, NONE);
        leaveWater(3);
        pickLaundry(2);
        robot.setLinearAccelParams(100, 40, 0);
        robot.straight(40, 5, COAST);

        robot.setLinearAccelParams(100, 0, 0);
        robot.straight(40, -9, COAST);

        robot.setLinearAccelParams(100, 20, 25);
        robot.arc(50, 100, -3.5, NONE);
        robot.setLinearAccelParams(100, 25, 25);
        robot.arcUnlim(25, -3.5, FORWARD, true);
        while(leftSensor.getReflected() < 80 && abs(robot.getAngle()) < 15)
            robot.arcUnlim(25, -3.5, FORWARD);   
        // robot.stop(COAST);
    }
}

void room::taskBall()
{
    if(roomOrientation == GREEN_YELLOW)
    {
        pickBall(1);
        robot.setLinearAccelParams(100, 0, 0);
        robot.arc(40, 47, 3, COAST);
        robot.straight(40, 4.7, COAST);
        pickBall(2);
        robot.arc(40, -115, 2, COAST);

        robot.straight(40, 17, COAST);
        leaveBall();

        robot.straight(40, -10, COAST);
        robot.arc(40, -115, 0, COAST);
        robot.setLinearAccelParams(100, 0, 30);
        robot.straight(40, 17, NONE);
    }
    else    //RED_BLUE
    {
        pickBall(1);
        robot.setLinearAccelParams(100, 0, 0);
        robot.arc(40, 47, -3, COAST);
        robot.straight(40, 4.7, COAST);
        pickBall(2);
        robot.arc(40, -117, -2, COAST);

        robot.straight(40, 17, COAST);
        leaveBall();

        robot.straight(40, -10, COAST);
        robot.arc(40, 115, 0, COAST);
        robot.setLinearAccelParams(100, 0, 30);
        robot.straight(40, 17, NONE);
    }
}

void room::taskBallLaundry()
{
    if(roomOrientation == GREEN_YELLOW)
    {
        pickLaundry(1);
        robot.setLinearAccelParams(100, 0, -25);
        robot.arc(35, -48, -8.5, NONE);
        
        robot.setLinearAccelParams(100, -30, 0);
        robot.arc(35, -48, -4, COAST);

        robot.setLinearAccelParams(100, 0, 40);
        robot.straight(40, 7, NONE);    
        pickLaundry(2);
        robot.setLinearAccelParams(100, 40, 0);
        robot.straight(40, 9.5, COAST);
        
        robot.setLinearAccelParams(100, 0, 0);
        robot.arc(40, 93, -3, COAST);

        robot.setLinearAccelParams(100, 0, 0);
        robot.straight(40, 7.5, BRAKE);
        pickBall(2);
        robot.setLinearAccelParams(100, 0, 0);
        robot.arc(40, 80, -3, COAST);

        robot.setLinearAccelParams(100, 0, 0);
        robot.straight(40, 21, COAST);
        leaveBall();

        robot.setLinearAccelParams(100, 0, 0);
        robot.straight(40, -9, COAST);
        robot.arc(40, -100, 0, COAST);
        robot.setLinearAccelParams(100, 0, 30);
        robot.straight(40, 20, COAST);
    }
    else    //RED_BLUE
    {
        pickLaundry(1);
        robot.setLinearAccelParams(100, 0, -25);
        robot.arc(35, -48, 8.5, NONE);
        
        robot.setLinearAccelParams(100, -30, 0);
        robot.arc(35, -48, 4, COAST);

        robot.setLinearAccelParams(100, 0, 40);
        robot.straight(40, 7, NONE);    
        pickLaundry(2);
        robot.setLinearAccelParams(100, 40, 0);
        robot.straight(40, 9.5, COAST);
        
        robot.setLinearAccelParams(100, 0, 0);
        robot.arc(40, 100, 3, COAST);

        robot.setLinearAccelParams(100, 0, 0);
        robot.straight(40, 7.5, BRAKE);
        pickBall(2);
        robot.setLinearAccelParams(100, 0, 0);
        robot.arc(40, 80, 3, COAST);

        robot.setLinearAccelParams(100, 0, 0);
        robot.straight(40, 21, COAST);
        leaveBall();

        robot.setLinearAccelParams(100, 0, 0);
        robot.straight(40, -9, COAST);
        robot.arc(40, 107, 0, COAST);
        robot.setLinearAccelParams(100, 0, 30);
        robot.straight(40, 20, COAST);
    }
}

void room::taskBoth()
{
    if(roomOrientation == GREEN_YELLOW)
    {
        pickLaundry(1);
        leaveWater(1);
        robot.setLinearAccelParams(100, 0, -25);
        robot.arc(35, -48, -8.5, NONE);
        
        robot.setLinearAccelParams(100, -30, 0);
        robot.arc(35, -48, -4, COAST);
        leaveWater(2);

        robot.setLinearAccelParams(100, 0, 40);
        robot.straight(40, 7, NONE);    
        pickLaundry(2);
        leaveWater(3);
        robot.setLinearAccelParams(100, 40, 0);
        robot.straight(40, 9.5, COAST);
        
        robot.setLinearAccelParams(100, 0, 0);
        robot.arc(40, 93, -3, COAST);

        robot.setLinearAccelParams(100, 0, 0);
        robot.straight(40, 7.5, BRAKE);
        pickBall(2);
        robot.setLinearAccelParams(100, 0, 0);
        robot.arc(40, 80, -3, COAST);

        robot.setLinearAccelParams(100, 0, 0);
        robot.straight(40, 21, COAST);
        leaveBall();

        robot.setLinearAccelParams(100, 0, 0);
        robot.straight(40, -9, COAST);
        robot.arc(40, -103, 0, COAST);
        robot.setLinearAccelParams(100, 0, 30);
        robot.straight(40, 20, COAST);
    }
    else
    {
        pickLaundry(1);
        leaveWater(1);
        robot.setLinearAccelParams(100, 0, -25);
        robot.arc(35, -48, 8.5, NONE);
        
        robot.setLinearAccelParams(100, -30, 0);
        robot.arc(35, -48, 4, COAST);
        leaveWater(2);

        robot.setLinearAccelParams(100, 0, 40);
        robot.straight(40, 7, NONE);    
        pickLaundry(2);
        leaveWater(3);
        robot.setLinearAccelParams(100, 40, 0);
        robot.straight(40, 9.5, COAST);
        
        robot.setLinearAccelParams(100, 0, 0);
        robot.arc(40, 98, 3, COAST);

        robot.setLinearAccelParams(100, 0, 0);
        robot.straight(40, 7.5, BRAKE);
        pickBall(2);
        robot.setLinearAccelParams(100, 0, 0);
        robot.arc(40, 80, 3, COAST);

        robot.setLinearAccelParams(100, 0, 0);
        robot.straight(40, 21, COAST);
        leaveBall();

        robot.setLinearAccelParams(100, 0, 0);
        robot.straight(40, -9, COAST);
        robot.arc(40, 103, 0, COAST);
        robot.setLinearAccelParams(100, 0, 30);
        robot.straight(40, 20, COAST);
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
    else if(task == BOTH)
    {
        taskBoth();
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
    else if(color == BLUE)
        return BOTH;
    else // if(color == GREEN)
        return BALL;
}

void inferYellowRoomTask()
{
    int waterTasks = 0;
    if(rooms[RED].getTask() == WATER)
        waterTasks++;
    if(rooms[GREEN].getTask() == WATER)
        waterTasks++;  
    if(rooms[BLUE].getTask() == WATER)
        waterTasks++;
    rooms[YELLOW].setTask(waterTasks == 2 ? GREEN : WHITE);
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

items findItem(ev3ys::colors color)
{
    switch(color)
    {
        case BLACK:
            return LAUNDRY_BLACK;
        case RED:
            return LAUNDRY_RED;
        case YELLOW:
            return LAUNDRY_YELLOW;
        default:
            return BOTTLE;   //Should not happen
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

colors analyzeFrequency(map<colors, int> appearances, colors base)
{
    colors result = base;
    int maxCount = 0;
    for(auto x: appearances)
    {
        if(x.second > maxCount)
        {
            maxCount = x.second;
            result = x.first;    
        }
    }
    return result;
}

colors scanLaundryBlock(colorSensor &scanner)
{
    scanner.setNormalisation(true);
    colors color = scanner.getColor();

    if(color == WHITE)
        return BLACK;

    return color;
}

colors scanCodeBlock(colorSensor &scanner)
{
    scanner.setNormalisation(true);
    colors color = scanner.getColor();

    if(color == BLACK)
        return WHITE;

    return color;
}

colors scanLaundryBasket(colorSensor &scanner)
{
    scanner.setNormalisation(false);
    colorspaceRGB rgb = scanner.getRGB();

    if(rgb.green > 3) return YELLOW;
    if(rgb.red > 3) return RED;
    return BLACK;
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
    colorspaceRGB rgb = sensor.getRGB();
    tslp_tsk(1);
    return rgb.red > 200 && rgb.green > 200 && rgb.blue > 200;
}

void turnToBasket(baskets current, baskets target)
{
    int turnDifference = target - current;
    if(turnDifference != 0)
        robot.arc(35, -36 * turnDifference, 0, COAST);
}


//Actual tasks start here

void startProcedure()
{
    DEBUGPRINT("\nStarting movement!!!\n");

    robot.setMode(CONTROLLED);
    robot.setLinearAccelParams(100, 0, 30);
    robot.straight(50, 15, NONE);

    //Get out of the start position
    currentDirection = NORTH;

}

void pickWater()
{
    DEBUGPRINT("\nPicking water bottles.\n");
    
    //Pick First Block
    robot.setLinearAccelParams(100, 40, 30);
    robot.straight(40, 5, NONE);
    act_tsk(WATER_GRABBER_TASK);
    tslp_tsk(1);
    robot.setLinearAccelParams(100, 30, 0);
    robot.straight(30, 2, COAST);
    DEBUGPRINT("First bottle of water has been loaded.\n");
    rampQueue.push(BOTTLE);

    //Pick Second Block
    robot.setLinearAccelParams(100, 0, 0);
    robot.arc(50, -43, -8.5, COAST);
    robot.setLinearAccelParams(100, 0, 30);
    robot.straight(50, 10, NONE);
    act_tsk(PICK_BLOCK_TASK);
    tslp_tsk(1);
    robot.setLinearAccelParams(100, 30, 0);
    robot.straight(30, 3, COAST);
    DEBUGPRINT("Second bottle of water has been loaded.\n");
    rampQueue.push(BOTTLE);

    //Get to TR intersection
    robot.setLinearAccelParams(100, 20, 20);
    robot.arc(40, 90, 3, COAST);

    robot.setLinearAccelParams(100, 20, 30);
    robot.straightUnlim(30, true);
    while(robot.getPosition() < 1) 
        robot.straightUnlim(30);
    while(rightSensor.getReflected() > 80)
        robot.straightUnlim(30);
    while(leftSensor.getReflected() > 80)
        robot.straightUnlim(30);
}
void pickWaterTriple()
{
    DEBUGPRINT("\nPicking all three water bottles (variations).\n");
        
    //Pick First Bottle
    robot.setLinearAccelParams(100, 40, 30);
    robot.straight(40, 5, NONE);
    act_tsk(WATER_GRABBER_TASK);
    tslp_tsk(1);
    robot.setLinearAccelParams(100, 30, 0);
    robot.straight(30, 2, COAST);
    DEBUGPRINT("First bottle of water has been loaded.\n");
    rampQueue.push(BOTTLE);

    //Pick Second Bottle (left)
    robot.setLinearAccelParams(100, 0, 0);
    robot.arc(50, -35, 8.5, COAST);
    robot.setLinearAccelParams(100, 0, 30);
    robot.straight(50, 10, NONE);
    act_tsk(WATER_GRABBER_TASK);
    tslp_tsk(1);
    robot.setLinearAccelParams(100, 30, 0);
    robot.straight(30, 3, COAST);
    DEBUGPRINT("Second bottle of water has been loaded.\n");
    rampQueue.push(BOTTLE);

    //Pick Third Bottle (right)
    robot.setLinearAccelParams(100, 0, 0);
    robot.arc(50, -82, -8.5, COAST);

    robot.setLinearAccelParams(100, 0, 30);
    robot.straight(50, 13, NONE);
    act_tsk(PICK_BLOCK_TASK);
    tslp_tsk(1);
    robot.setLinearAccelParams(100, 30, 0);
    robot.straight(30, 3, BRAKE);
    DEBUGPRINT("Third bottle of water has been loaded.\n");
    rampQueue.push(BOTTLE);

    //Get to TR intersection
    robot.setLinearAccelParams(100, 20, 20);
    robot.arc(40, 90, 3, COAST);

    robot.setLinearAccelParams(100, 20, 30);
    robot.straightUnlim(30, true);
    while(robot.getPosition() < 1) 
        robot.straightUnlim(30);
    while(rightSensor.getReflected() > 80)
        robot.straightUnlim(30);
    while(leftSensor.getReflected() > 80)
        robot.straightUnlim(30);
}
void pickWaterLast()
{
    DEBUGPRINT("\nPicking last water bottle (remaining) at the end (variation).\n");

    robot.setLinearAccelParams(100, 40, 0);
    robot.straight(40, 5, COAST);

    grabber.setMode(REGULATED);
    grabber.moveDegrees(-700, 350, COAST, false);
    
    robot.setLinearAccelParams(100, 0, 0);
    robot.arc(50, -35, 8.5, NONE);
    act_tsk(OPEN_GRABBER_TASK);
    tslp_tsk(1);
    robot.setLinearAccelParams(100, 0, 30);
    robot.straight(50, 10, NONE);
    act_tsk(PICK_BLOCK_TASK);
    tslp_tsk(1);
    robot.setLinearAccelParams(100, 30, 0);
    robot.straight(30, 3, COAST);
    rampQueue.push(BOTTLE);


    robot.setLinearAccelParams(100, 0, 0);
    robot.arc(50, -140, 3, COAST);

    currentDirection = SOUTH;
}

void scanLaundryBaskets()
{
    DEBUGPRINT("\nScanning laundry baskets.\n");

    stopScanning = false;
    act_tsk(BASKET_SCAN_TASK);
    tslp_tsk(1);

    robot.setMode(CONTROLLED);
    robot.setLinearAccelParams(100, 0, 0);
    robot.arc(45, 30, -8.5, COAST);
    robot.arc(45, -30, -8.5, COAST);
    robot.arc(45, 30, 8.5, COAST);
    robot.arc(45, -30, 8.5, COAST);
    stopScanning = true;
    tslp_tsk(1);
    robot.arc(45, -180, 0, COAST);

    //Calculating Colors
    colors temp[2];
    temp[0] = laundryBaskets[BASKET_LEFT];
    temp[1] = laundryBaskets[BASKET_RIGHT];

    //Simple fix for if there is a mistake in the readings so that the robot can finish the mission (wrongly :( )
    if(temp[0] == BLACK && temp[1] == BLACK)
    {
        DEBUGPRINT("Mistake in laundry basket scanning !!!");
        temp[1] = RED;
    }
    else if(temp[0] == temp[1])
    {
        DEBUGPRINT("Mistake in laundry basket scanning !!!");
        temp[1] = (temp[0] == RED) ? YELLOW : RED;
    }

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

    robot.setMode(CONTROLLED);
    robot.setLinearAccelParams(100, 0, 0);

    //Detect possible mistakes (color detection issues i.e. duplicates in the queue or less than 3 items + laundry basket scanning problems duplicates) and change them to at least get some points and not lose the run completely
    //TODO: easy fix is to change the scanning of the baskets that way nothing terrible will happen. ->DONE


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
            //2  back then forward
            robot.straight(30, -4, COAST);
            emptyRampLaundry();
            robot.straight(30, 4, COAST);
        }
        else
        {
            //4 back then forward
            robot.straight(30, -5, COAST);
            emptyRampLaundry();
            robot.straight(30, 5, COAST);
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
    robot.setLinearAccelParams(100, 0, 0);
    act_tsk(END_TASK);
    tslp_tsk(1);
    robot.arc(45, 45, 2, BRAKE);
}
