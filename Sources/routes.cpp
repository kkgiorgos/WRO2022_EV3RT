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

    //"Efficiency" edges
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

    resetLifo();
    lifo.setPIDparams(KP * 1.2, slowKI * 0.7, KD*1.5, 1);
    lifo.distance(robot.cmToTacho(30), 10, NONE);
    setLifoSlow();
    lifo.setAccelParams(150, 20, 20);
    lifo.distance(20, 3, NONE);
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

    robot.setLinearAccelParams(100, 35, 45);
    robot.arc(45, 30, 15, NONE);
    robot.setLinearAccelParams(100, 45, 25);
    robot.arc(45, 20, 40, NONE);
    robot.setLinearAccelParams(100, 35, 35);
    robot.arcUnlim(45, 15, FORWARD, true);
    colors current = BLACK;
    map<colors, int> appearances;
    while(robot.getAngle() < 40)
    {
        if((current = scanCodeBlock(leftScanner)) != BLACK)
        {
            appearances[current]++;
        }
        robot.arcUnlim(45, 15, FORWARD, false);
    }
    int maxCount = 0;
    for(auto x: appearances)
    {
        if(x.second > maxCount)
        {
            maxCount = x.second;
            current = x.first;    
        }
    }
    rooms[RED].setTask(current);
    display.resetScreen();
    display.format("%  \n")%static_cast<int>(current);

    robot.setLinearAccelParams(100, 20, 45);
    robot.straightUnlim(45, true);
    do{
        leftSensor.getReflected();
        robot.straightUnlim(45);
    }while(!leftSensor.getLineDetected());
    robot.resetPosition();
    while(robot.getPosition() < 1) robot.straightUnlim(45);
    while(leftSensor.getReflected() > 80) robot.straightUnlim(45);
    robot.setLinearAccelParams(100, 45, 0);
    robot.straight(45, 12, COAST);
    
    robot.setLinearAccelParams(100, 0, -25);
    robot.arc(45, -75, 3.5, COAST);
    robot.setLinearAccelParams(100, -25, -25);
    robot.arcUnlim(25, 3.5, BACKWARD, true);
    while(leftSensor.getReflected() < 50)
        robot.arcUnlim(25, 3.5, BACKWARD, false);

    resetLifo();
    setLifoLeftExtreme();
    lifo.distance(robot.cmToTacho(20), 2, NONE);
    lifo.distance(robot.cmToTacho(30), 3, NONE);
    setLifoLeft();
    while(rightSensor.getReflected() < 60)
        executeLifoLeftUnlim(robot.cmToTacho(30));

    current = BLACK;
    appearances.clear();
    robot.resetPosition();
    while(robot.getPosition() < 8)
    {
        if((current = scanCodeBlock(rightScanner)) != BLACK)
        {
            appearances[current]++;
        }
        executeLifoLeftUnlim(robot.cmToTacho(30));
    }
    maxCount = 0;
    for(auto x: appearances)
    {
        if(x.second > maxCount)
        {
            maxCount = x.second;
            current = x.first;    
        }
    }
    rooms[GREEN].setTask(current);
    display.format("%  \n")%static_cast<int>(current);

    return EAST;
}

orientation L_S(orientation dir)
{
    DEBUGPRINT("\nL_S\n");
    
    resetLifo();
    lifo.setPIDparams(KP * 1.2, slowKI * 0.7, KD*1.5, 1);
    lifo.distance(robot.cmToTacho(30), 10, NONE);
    setLifoSlow();
    lifo.setAccelParams(150, 20, 20);
    lifo.distance(20, 3, NONE);

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
    
    resetLifo();
    setLifoRightExtreme();
    lifo.distance(robot.cmToTacho(30), 8, NONE);
    
    lifo1WhiteLineRightSlow(35, 2, 35, NONE);

    robot.setMode(CONTROLLED);
    robot.setLinearAccelParams(150, 35, 0);
    robot.straight(35, 7, NONE);

    robot.setLinearAccelParams(100, -35, -35);
    robot.arc(45, -80, 4, NONE);
    robot.arcUnlim(35, 4, BACKWARD, true);
    while(rightSensor.getReflected() > 60)
        robot.arcUnlim(35, 4, BACKWARD);  

    return WEST;
}
orientation CL_L(orientation dir)
{
    DEBUGPRINT("\nCL_L\n");

    timer t;

    resetLifo();
    lifo.distance(robot.cmToTacho(45), 10, NONE);
    lifo.lines(robot.cmToTacho(45), 1, NONE);
    robot.resetPosition();
    t.reset();
    lifo.distance(robot.cmToTacho(45), 22, NONE);
    
    double speed = robot.getPosition() / t.secElapsed();
    robot.setMode(CONTROLLED);
    robot.setLinearAccelParams(150, speed, speed);
    robot.straight(speed, 6, NONE);
    robot.setLinearAccelParams(100, speed, speed);
    robot.arc(45, 90, 17.5, COAST);
    // robot.setMode(REGULATED);
    // robot.arc(40, 90, 18, NONE);

    resetLifo();
    lifo.setPIDparams(KP * 1.2, slowKI * 0.7, KD*1.5, 1);
    lifo.distance(robot.cmToTacho(30), 10, NONE);
    setLifoSlow();
    lifo.setAccelParams(150, 20, 20);
    lifo.distance(20, 3, NONE);
    lifo.lines(20, 1, BRAKE);

    return SOUTH;
}

orientation CR_CL(orientation dir)
{
    DEBUGPRINT("\nCR_CL\n");

    resetLifo();
    setLifoRightExtreme();
    lifo.distance(robot.cmToTacho(30), 10, NONE);
    setLifoRight();
    lifo.unlimited(robot.cmToTacho(45), true);
    while(leftSensor.getReflected() < 60)
        executeLifoRightUnlim(robot.cmToTacho(45));
    robot.resetPosition();
    timer t;
    while(robot.getPosition() < 27)
        executeLifoRightUnlim(robot.cmToTacho(45));
    double speed = robot.getPosition() / t.secElapsed();
    robot.setLinearAccelParams(100, speed, speed);
    robot.straight(speed, 32, NONE);    
    setLifoRight();
    while(leftSensor.getReflected() < 60)
        executeLifoRightUnlim(robot.cmToTacho(45));
    robot.resetPosition();
    while(robot.getPosition() < 20)
        executeLifoRightUnlim(robot.cmToTacho(45));

    resetLifo();
    setLifoRightExtreme();
    lifo.distance(robot.cmToTacho(30), 5, NONE);
    setLifoRight();
    while(leftSensor.getReflected() < 60)
        executeLifoRightUnlim(robot.cmToTacho(30));


    colors current = BLACK;
    map<colors, int> appearances;
    robot.resetPosition();
    while(robot.getPosition() < 8)
    {
        if((current = scanCodeBlock(leftScanner)) != BLACK)
        {
            appearances[current]++;
        }
        executeLifoRightUnlim(robot.cmToTacho(30));
    }
    int maxCount = 0;
    for(auto x: appearances)
    {
        if(x.second > maxCount)
        {
            maxCount = x.second;
            current = x.first;    
        }
    }
    rooms[BLUE].setTask(current);
    display.format("%  \n")%static_cast<int>(current);

    //Infer Yellow Room's task.
    int waterTasks = 0;
    if(rooms[RED].getTask() == WATER)
        waterTasks++;
    if(rooms[GREEN].getTask() == WATER)
        waterTasks++;  
    if(rooms[BLUE].getTask() == WATER)
        waterTasks++;
    rooms[YELLOW].setTask(waterTasks == 2 ? GREEN : WHITE);
    
    return WEST;
}
orientation CR_FR(orientation dir)
{
    DEBUGPRINT("\nCR_FR\n");

    resetLifo();
    setLifoLeftExtreme();
    lifo.distance(robot.cmToTacho(30), 8, NONE);
    
    lifo1WhiteLineLeftSlow(35, 2, 35, NONE);

    robot.setMode(CONTROLLED);
    robot.setLinearAccelParams(150, 35, 0);
    robot.straight(35, 7, NONE);

    robot.setLinearAccelParams(100, -35, -35);
    robot.arc(45, -80, -4, NONE);
    robot.arcUnlim(35, -4, BACKWARD, true);
    while(leftSensor.getReflected() > 60)
        robot.arcUnlim(35, -4, BACKWARD); 

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

    // robot.setMode(REGULATED);
    // robot.arc(30, 90, -7.5, NONE);

    robot.setLinearAccelParams(100, 30, 30);
    robot.arc(45, 90, -8.5, COAST);

    resetLifo();
    lifo.setPIDparams(KP * 1.2, slowKI * 0.7, KD*1.5, 1);
    lifo.distance(robot.cmToTacho(30), 5, NONE);
    
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

    resetLifo();
    setLifoRightExtreme();
    lifo.distance(robot.cmToTacho(30), 10, NONE);
    setLifoSlow();
    setLifoRight(true);
    lifo.setAccelParams(100, 20, 20);
    lifo.distance(20, 4, NONE);

    return SOUTH;
}

orientation FR_CR(orientation dir)
{
    DEBUGPRINT("\nFR_CR\n");

    robot.setMode(REGULATED);
    robot.arc(35, 30, 8.5, NONE); 
    robot.arc(35, 60, 19.5, NONE);
    robot.arcUnlim(35, 19.5, FORWARD, true);
    while(rightSensor.getReflected() < 60)
        robot.arcUnlim(35, 19.5, FORWARD);

    robot.setMode(CONTROLLED);

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

    resetLifo();
    setLifoLeftExtreme();
    lifo.distance(robot.cmToTacho(30), 10, NONE);
    setLifoSlow();
    setLifoLeft(true);
    lifo.setAccelParams(100, 20, 20);
    lifo.distance(20, 4, NONE);
    
    return SOUTH;
}

orientation YR_FL(orientation dir)
{
    DEBUGPRINT("\nYR_FL\n");

    resetLifo();
    setLifoRightExtreme();
    lifo.distance(robot.cmToTacho(30), 10, NONE);
    setLifoRight();
    while(leftSensor.getReflected() < 60)
        executeLifoRightUnlim(robot.cmToTacho(30));

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

    resetLifo();
    setLifoLeftExtreme();
    lifo.distance(robot.cmToTacho(30), 10, NONE);
    setLifoLeft();
    while(rightSensor.getReflected() < 60)
        executeLifoLeftUnlim(robot.cmToTacho(30));

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

    resetLifo();
    setLifoLeftExtreme();
    lifo.distance(robot.cmToTacho(30), 10, NONE);
    setLifoLeft();
    while(!rightSensor.getLineDetected())
        executeLifoLeftUnlim(robot.cmToTacho(30));
    robot.resetPosition();
    while(robot.getPosition() < 5)
        executeLifoLeftUnlim(robot.cmToTacho(30));

    resetLifo();
    setLifoLeftExtreme();
    lifo.distance(robot.cmToTacho(30), 10, NONE);
    setLifoSlow();
    setLifoLeft(true);
    lifo.setAccelParams(100, 20, 20);
    lifo.distance(20, 5, NONE);

    return NORTH;
}

orientation GR_RR(orientation dir)
{
    DEBUGPRINT("\nGR_RR\n");

    resetLifo();
    setLifoRightExtreme();
    lifo.distance(robot.cmToTacho(30), 10, NONE);
    setLifoRight();
    while(!leftSensor.getLineDetected())
        executeLifoRightUnlim(robot.cmToTacho(30));
    robot.resetPosition();
    while(robot.getPosition() < 5)
        executeLifoRightUnlim(robot.cmToTacho(30));

    resetLifo();
    setLifoRightExtreme();
    lifo.distance(robot.cmToTacho(30), 10, NONE);
    setLifoSlow();
    setLifoRight(true);
    lifo.setAccelParams(100, 20, 20);
    lifo.distance(20, 5, NONE);

    return NORTH;
}
