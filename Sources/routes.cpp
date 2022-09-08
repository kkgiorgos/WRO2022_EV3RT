#include "routes.h"
#include "methods.h"
#include "tasks.h"
#include "ev3ys.h"
#include "globalRobot.h"
#include <cstdlib>

using namespace std;
using namespace ev3ys;
using namespace ev3cxx;

void addEdge(int u, int v)
{
    graph[u].push_back(v);
    graph[v].push_back(u);
}

void graphInit()
{
    addEdge(S, W);
    addEdge(S, L);
    
    addEdge(W, CL);
    addEdge(W, CR);
    
    addEdge(L, CL);
    addEdge(L, CR);
    
    addEdge(CL, FL);
    addEdge(CL, CR);
    
    addEdge(CR, FR);
    
    addEdge(FL, YR);
    addEdge(FL, BR);

    addEdge(FR, RR);
    addEdge(FR, GR);

    addEdge(BR, YR);
    addEdge(GR, RR);
}

int dijkstra(int source, int target, vector<int> *path)
{
    path->clear();
    int dist[V];
    int parent[V];
    priority_queue< weightedVertex, vector <weightedVertex> , greater<weightedVertex> > Q;
    for(int i = 0; i < V; i++)
    {
        dist[i] = INT_MAX;
        parent[i] = -1;
    }
    dist[source] = 0;
    Q.push(make_pair(0, source));

    while(!Q.empty())
    {
        int m = Q.top().second;
        Q.pop();
        for(auto w : graph[m])
        {
            if(dist[w] == INT_MAX)
            {
                dist[w] = dist[m] + 1;
                Q.push(make_pair(dist[w], w));
                parent[w] = m;
            }
            else
            {
                if(dist[w] > dist[m] + 1)
                {
                    dist[w] = dist[m] + 1;
                    parent[w] = m;
                }
            }
        }
    }
    vector<int> tempPath;
    for(int v = target; v != -1; v = parent[v])
    {
        tempPath.push_back(v);
    }
    while(tempPath.back() != target)
    {
        path->push_back(tempPath.back());
        tempPath.pop_back();
    }
    path->push_back(tempPath.back());
    tempPath.pop_back();
    return dist[target];
}

void constructRoute(routeFunc *route, vector<int> *pathNodes, int distance)
{
    for (int i = 0; i < distance; i++)
    {
        int firstNode = (*pathNodes)[i];
        int secondNode = (*pathNodes)[i+1];
        
        if(firstNode == S)
        {
            if(secondNode == W) route[i] = S_W;
        }
        else if(firstNode == W)
        {
            if(secondNode == CL) route[i] = W_CL;
            else if(secondNode == CR) route[i] = W_CR;
        }
        else if(firstNode == L)
        {
            if(secondNode == S) route[i] = L_S;
        }
        else if(firstNode == CL)
        {
            if(secondNode == CR) route[i] = CL_CR;
            else if(secondNode == FL) route[i] = CL_FL;
            else if(secondNode == L) route[i] = CL_L;
        }
        else if(firstNode == CR)
        {
            if(secondNode == CL) route[i] = CR_CL;
            else if(secondNode == FR) route[i] = CR_FR;
            else if(secondNode == L) route[i] = CR_L;
        }
        else if(firstNode == FL)
        {
            if(secondNode == CL) route[i] = FL_CL;
            else if(secondNode == YR) route[i] = FL_YR;
            else if(secondNode == BR) route[i] = FL_BR;
        }
        else if(firstNode == FR)
        {
            if(secondNode == CR) route[i] = FR_CR;
            else if(secondNode == RR) route[i] = FR_RR;
            else if(secondNode == GR) route[i] = FR_GR;
        }
        else if(firstNode == YR)
        {
            if(secondNode == FL) route[i] = YR_FL;
        }
        else if(firstNode == BR)
        {
            if(secondNode == FL) route[i] = BR_FL;
            else if(secondNode == YR) route[i] = BR_YR;
        }
        else if(firstNode == RR)
        {
            if(secondNode == FR) route[i] = RR_FR;
        }
        else if(firstNode == GR)
        {
            if(secondNode == FR) route[i] = GR_FR;
            else if(secondNode == RR) route[i] = GR_RR;
        }
    }
}

orientation executeRoute(routeFunc *route, int distance, orientation dir)
{
    for(int i = 0 ; i < distance; i++)
    {
        dir = route[i](dir);
    }
    return dir;
}

void fullRoute(int source, int target, orientation dir)
{
    vector<int> path;
    int distance = dijkstra(source, target, &path);
    DEBUGPRINT("\n\nNew Route with distance: %d\n", distance);
    routeFunc route[distance];
    constructRoute(route, &path, distance);
    currentDirection = executeRoute(route, distance, dir);
    currentPos = static_cast<matPos>(target);
}

void fullRouteStandard(int target)
{
    fullRoute(currentPos, target, currentDirection);
}

void standardTurn(orientation start, orientation finish)
{
    int turnDifference = finish - start;
    currentDirection = finish;
    if(turnDifference == -1 || turnDifference == 3)
    {
        DEBUGPRINT("Turning left 90\n");
        leftTurn();
    }
    else if(turnDifference == 1 || turnDifference == -3)
    {
        DEBUGPRINT("Turning right 90\n");
        rightTurn();
    }
    else if(turnDifference == -2 || turnDifference == 2)
    {
        DEBUGPRINT("Reversing\n");
        reverse();
    }
}

void specialTurn(orientation start, orientation finish)
{
    int turnDifference = finish - start;
    currentDirection = finish;
    if(turnDifference == -1 || turnDifference == 3)
    {
        DEBUGPRINT("Turning left 90 no align\n");
        leftTurn(false, false);
    }
    else if(turnDifference == 1 || turnDifference == -3)
    {
        DEBUGPRINT("Turning right 90 no align\n");
        rightTurn(false, false);
    }
    else if(turnDifference == -2 || turnDifference == 2)
    {
        DEBUGPRINT("Reversing no align\n");
        reverse(false, false);
    }
}

orientation S_W(orientation dir)
{
    DEBUGPRINT("\nS_W\n");

    timer t;
    resetLifo();
    lifo.distance(400, 5, NONE);
    robot.resetPosition();
    t.reset();
    lifo.distance(500, 2, NONE);
    double currentVelocity = robot.getPosition() / t.secElapsed();
    setLifoSlow();
    lifo.setAccelParams(250, currentVelocity, 20);
    lifo.distance(currentVelocity, 7, NONE);
    lifo.setAccelParams(200, 20, 20);
    lifo.lines(20, 1, NONE);

    return NORTH;
}

orientation W_CL(orientation dir)
{
    DEBUGPRINT("\nW_CL\n");
    //MAYBE USELESS
    return WEST;
}
orientation W_CR(orientation dir)
{
    DEBUGPRINT("\nW_CR\n");

    robot.setLinearAccelParams(100, 0, 35);
    robot.arc(50, 93, 4.5, NONE);
    robot.setLinearAccelParams(100, 35, 40);
    robot.straight(40, 25, NONE);
    alignOnMove(40);
    robot.setLinearAccelParams(100, 40, 30);
    robot.straight(30, 10, NONE);
    robot.setLinearAccelParams(100, 30, 30);
    robot.straightUnlim(30, true);    
    while(leftSensor.getHSV().saturation < 30)
        robot.straightUnlim(30);
    robot.setLinearAccelParams(100, 30, 0);
    robot.straight(30, 6.5);

    robot.setAngularAccelParams(1000, 0, 50);
    robot.turn(300, 90);

    colors current;
    robot.setMode(CONTROLLED);
    robot.setLinearAccelParams(100, 0, 40);
    robot.straightUnlim(40, true);
    while((current = scanCodeBlock(leftScanner)) == BLACK)
        robot.straightUnlim(40);

    rooms[RED].setTask(scanCodeBlock(leftScanner));
    display.resetScreen();
    display.format("%  \n")%static_cast<int>(scanCodeBlock(leftScanner));

    robot.setLinearAccelParams(100, 40, 50);
    robot.straight(50, 15, NONE);
    robot.straightUnlim(50, true);
    while(leftSensor.getReflected() > 10)
        robot.straightUnlim(50);
    robot.setLinearAccelParams(100, 50, 0);
    robot.straight(50, 11);

    robot.setAngularAccelParams(1000, 0, 50);
    robot.turn(300, -90);

    lifo1WhiteLineLeftSlow(20, 3, 20, NONE);
    robot.setLinearAccelParams(200, 20, 30);
    robot.straightUnlim(30, true);
    while((current = scanCodeBlock(rightScanner)) == BLACK)
        robot.straightUnlim(30);
    rooms[GREEN].setTask(scanCodeBlock(rightScanner));
    display.format("%  \n")%static_cast<int>(scanCodeBlock(rightScanner));
    while(robot.getPosition() < 8)
        robot.straightUnlim(30);

    resetLifo();

    return EAST;
}

orientation L_S(orientation dir)
{
    DEBUGPRINT("\nL_S\n");
    
    lifo.distance(50, 12, NONE);

    return NO;
}

orientation CL_CR(orientation dir)
{
    DEBUGPRINT("\nCL_CR\n");
    //MAYBE USELESS
    return EAST;
}
orientation CL_FL(orientation dir)
{
    DEBUGPRINT("\nCL_FL\n");
    
    setLifoRight();
    lifo.setAccelParams(250, 50, 50); 
    lifo.unlimited(50, true);
    do
    {
        leftSensor.getReflected();
        rightSensor.getReflected();
        executeLifoRightUnlim();
    }
    while(leftSensor.getLineDetected() || rightSensor.getLineDetected());

    do
    {
        leftSensor.getReflected();
        rightSensor.getReflected();
        executeLifoRightUnlim();
    }
    while(!leftSensor.getLineDetected() && !rightSensor.getLineDetected());

    resetLifo();

    return WEST;
}
orientation CL_L(orientation dir)
{
    DEBUGPRINT("\nCL_L\n");
    
    standardTurn(dir, EAST);
    lifo1LineDist(20);
    lifo.distance(50, 20, NONE);
    robot.setMode(REGULATED);
    robot.tank(robot.cmToTacho(50), robot.cmToTacho(50), robot.cmToTacho(6));
    robot.setMode(CONTROLLED);
    robot.setLinearAccelParams(200, 50, 0);
    robot.arc(50, 90, 20, NONE);
    
    lifo1LineDist(10);

    return SOUTH;
}

orientation CR_CL(orientation dir)
{
    DEBUGPRINT("\nCR_CL\n");

    setLifoRight();
    lifo.initializeMotionMode(CONTROLLED);
    lifo.setAccelParams(200, 50, 50);
    lifo.unlimited(50, true);
    while(robot.getPosition() < 55)
        executeLifoRightUnlim();

    robot.setMode(CONTROLLED);
    robot.setLinearAccelParams(200, 60, 60);
    robot.straight(60, 32, NONE);

    lifo.unlimited(50, true);
    while(robot.getPosition() < 15)
        executeLifoRightUnlim();

    lifo.unlimited(50, true);
    do
    {
        leftSensor.getReflected();
        rightSensor.getReflected();
        executeLifoRightUnlim();
    }
    while(leftSensor.getLineDetected() || rightSensor.getLineDetected());

    do
    {
        leftSensor.getReflected();
        rightSensor.getReflected();
        executeLifoRightUnlim();
    }
    while(!leftSensor.getLineDetected() && !rightSensor.getLineDetected());

    lifo.unlimited(50, true);
    while(robot.getPosition() < 15)
        executeLifoRightUnlim();

    colors current;
    lifo.unlimited(50, true);
    while((current = scanCodeBlock(leftScanner)) == BLACK)
        executeLifoRightUnlim();
    rooms[BLUE].setTask(scanCodeBlock(leftScanner));

    //Infer Yellow Room's task.
    int waterTasks = 0;
    if(rooms[RED].getTask() == WATER)
        waterTasks++;
    if(rooms[GREEN].getTask() == WATER)
        waterTasks++;  
    if(rooms[BLUE].getTask() == WATER)
        waterTasks++;
    rooms[YELLOW].setTask(waterTasks == 2 ? GREEN : WHITE);
    
    display.format("%  \n")%static_cast<int>(scanCodeBlock(leftScanner));

    lifo.unlimited(50, true);
    while(robot.getPosition() < 10)
        executeLifoRightUnlim();

    resetLifo();

    return WEST;
}
orientation CR_FR(orientation dir)
{
    DEBUGPRINT("\nCR_FR\n");

    lifo1WhiteLineLeftSlow(30, 12);

    return EAST;
}
orientation CR_L(orientation dir)
{
    DEBUGPRINT("\nCR_L\n");
    //MAYBE USELESS
    return SOUTH;
}

orientation FL_CL(orientation dir)
{
    DEBUGPRINT("\nFL_CL\n");

    lifo1LineDist(5);
    
    return EAST;
}
orientation FL_YR(orientation dir)
{
    DEBUGPRINT("\nFL_YR\n");

    setLifoLeft();

    lifo.setAccelParams(600, 40, 40);
    lifo.distance(40, 10, NONE);
    
    lifo.unlimited(40, true);
    while(!detectColorLine(rightSensor, YELLOW))
        lifo.unlimited(40);

    return NORTH;
}
orientation FL_BR(orientation dir)
{
    DEBUGPRINT("\nFL_BR\n");

    robot.stop(BRAKE);
    robot.setMode(REGULATED);
    robot.arc(800, -87, 2, BRAKE);

    setLifoRightExtreme();
    robot.resetPosition();
    lifo.setAccelParams(600, 0, 50);
    lifo.unlimited(50, true);
    
    while(robot.getPosition() < 7)
        executeLifoRightUnlim();

    setLifoRight();
    while(!detectColorLine(leftSensor, BLUE))
        executeLifoRightUnlim();

    resetLifo();

    return SOUTH;
}

orientation FR_CR(orientation dir)
{
    DEBUGPRINT("\nFR_CR\n");

    setLifoRightExtreme();
    lifo.setAccelParams(250, 0, 50); 
    lifo.unlimited(50, true);
    do
    {
        leftSensor.getReflected();
        rightSensor.getReflected();
        executeLifoRightUnlim();
    }
    while(leftSensor.getLineDetected() || rightSensor.getLineDetected());

    do
    {
        leftSensor.getReflected();
        rightSensor.getReflected();
        executeLifoRightUnlim();
    }
    while(!leftSensor.getLineDetected() && !rightSensor.getLineDetected());

    resetLifo();

    return WEST;
}
orientation FR_RR(orientation dir)
{
    DEBUGPRINT("\nFR_RR\n");

    // setLifoRight();

    // lifo.setAccelParams(600, 40, 40);
    // lifo.distance(40, 10, NONE);
    
    // lifo.unlimited(40, true);
    // while(!detectColorLine(leftSensor, RED))
    //     lifo.unlimited(40);

    return NORTH;
}
orientation FR_GR(orientation dir)
{
    DEBUGPRINT("\nFR_GR\n");

    // robot.stop(BRAKE);
    // robot.setMode(REGULATED);
    // robot.arc(800, -85, -1.5, BRAKE); //1.6

    // setLifoLeft();
    // lifo.setAlignMode(true);
    // lifo.initializeMotionMode(UNREGULATED);
    // lifo.seconds(0, 0.2, NONE);
    // resetLifo();
    
    // setLifoLeftExtreme();
    // robot.resetPosition();
    // lifo.setAccelParams(600, 0, 50);
    // lifo.unlimited(50, true);
    
    // while(robot.getPosition() < 7)
    //     executeLifoLeftUnlim();

    // setLifoLeft();
    // while(!detectColorLine(rightSensor, GREEN))
    //     executeLifoLeftUnlim();

    // resetLifo();


    robot.setLinearAccelParams(100, 20, 0);
    robot.straight(20, 3.7);
    robot.setAngularAccelParams(1000, 0, 50);
    robot.turn(300, 90);

    lifo.initializeMotionMode(CONTROLLED);
    lifo.setDoubleFollowMode("SL", "62");
    lifo.setAlignMode(true);
    lifo.setPIDparams(slowKP*3, slowKI*1.8, slowKD*5, 1);
    lifo.setAccelParams(200, 20, 20);
    lifo.distance(20, 7, NONE);
    lifo.setPIDparams(slowKP*2, slowKI*2, slowKD*2, 1);
    lifo.distance(20, 3, NONE);

    resetLifo();

    return SOUTH;
}

orientation YR_FL(orientation dir)
{
    DEBUGPRINT("\nYR_FL\n");

    lifo.setAccelParams(200, 0, 50);
    setLifoRightExtreme();
    lifo.unlimited(50, true);
    do
    {
        executeLifoRightUnlim();
    }
    while(!leftSensor.getLineDetected() && !rightSensor.getLineDetected());
    
    
    setLifoLeft();
    do
    {
        executeLifoRightUnlim();
    }
    while(leftSensor.getLineDetected() || rightSensor.getLineDetected());

    while(leftSensor.getReflected() > 50 && rightSensor.getReflected() > 50)
        executeLifoRightUnlim();

    leftTurn(false, true);

    resetLifo();

    return SOUTH;
}
orientation BR_FL(orientation dir)
{
    DEBUGPRINT("\nBR_FL\n");

    setLifoLeft();
    lifo.setAccelParams(250, 0, 50); 
    lifo.unlimited(50, true);
    do
    {
        leftSensor.getReflected();
        rightSensor.getReflected();
        executeLifoLeftUnlim();
    }
    while(leftSensor.getLineDetected() || rightSensor.getLineDetected());

    do
    {
        leftSensor.getReflected();
        rightSensor.getReflected();
        executeLifoLeftUnlim();
    }
    while(!leftSensor.getLineDetected() && !rightSensor.getLineDetected());

    resetLifo();

    return NORTH;
}
orientation RR_FR(orientation dir)
{
    DEBUGPRINT("\nRR_FR\n");

    lifo.setAccelParams(200, 0, 50);
    setLifoLeftExtreme();
    lifo.unlimited(50, true);
    do
    {
        executeLifoLeftUnlim();
    }
    while(!leftSensor.getLineDetected() && !rightSensor.getLineDetected());
    
    
    setLifoLeft();
    do
    {
        executeLifoLeftUnlim();
    }
    while(leftSensor.getLineDetected() || rightSensor.getLineDetected());

    while(leftSensor.getReflected() > 50 && rightSensor.getReflected() > 50)
        executeLifoLeftUnlim();

    robot.setLinearAccelParams(200, 40, 0);
    robot.straight(40, 9);
    rightTurn(false, false);

    resetLifo();

    return SOUTH;
}
orientation GR_FR(orientation dir)
{
    DEBUGPRINT("\nGR_FR\n");

    // setLifoRight();
    // lifo.setAccelParams(250, 0, 50); 
    // lifo.distance(50, 15, NONE);
   
    // robot.setMode(CONTROLLED);
    // robot.setLinearAccelParams(200, 40, 40);
    // robot.straightUnlim(40, true);
    // do
    // {
    //     leftSensor.getReflected();
    //     rightSensor.getReflected();
    //     robot.straightUnlim(40);
    // }
    // while(!leftSensor.getLineDetected() && !rightSensor.getLineDetected());
    
    // do
    // {
    //     leftSensor.getReflected();
    //     rightSensor.getReflected();
    //     robot.straightUnlim(40);
    // }
    // while(leftSensor.getLineDetected() || rightSensor.getLineDetected());

    // while(leftSensor.getReflected() > 50 && rightSensor.getReflected() > 50)
    //     robot.straightUnlim(40);

    // while(leftSensor.getReflected() < 50)
    //     robot.straightUnlim(40);

    // robot.straight(40, 1, NONE);

    // resetLifo();

    return NORTH;
}

orientation BR_YR(orientation dir)
{
    DEBUGPRINT("\nBR_YR\n");

    lifo.setAccelParams(200, 0, 50);
    setLifoLeftExtreme();
    lifo.unlimited(50, true);
    while(robot.getPosition() < 10)
        executeLifoLeftUnlim();
    setLifoLeft();
    robot.resetPosition();
    while(robot.getPosition() < 20)
        executeLifoLeftUnlim();
    while(!detectColorLine(rightSensor, YELLOW))
        executeLifoLeftUnlim();

    resetLifo();

    return NORTH;
}

orientation GR_RR(orientation dir)
{
    DEBUGPRINT("\nGR_RR\n");

    lifo.setAccelParams(200, 0, 50);
    setLifoRightExtreme();
    lifo.unlimited(50, true);
    while(robot.getPosition() < 10)
        executeLifoRightUnlim();
    setLifoRight();
    robot.resetPosition();
    while(robot.getPosition() < 20)
        executeLifoRightUnlim();
    while(!detectColorLine(leftSensor, RED))
        executeLifoRightUnlim();

    resetLifo();

    return NORTH;
}
