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
    //Main network
    addEdge(S, W);
    addEdge(W, G);
    addEdge(G, R);
    addEdge(R, B);
    addEdge(B, Y);
    addEdge(Y, L);
    addEdge(L, S);

    //Full (surprise) network
    addEdge(M, TM);
    addEdge(M, BM);
    addEdge(M, CL1);
    addEdge(M, CR1);
    addEdge(CL1, TL);
    addEdge(CL1, BL);
    addEdge(CR1, TR);
    addEdge(CR1, BR);
    addEdge(CL1, CL2);
    addEdge(CR1, CR2);
    addEdge(CL2, CL3);
    addEdge(CR2, CR3);
    addEdge(CL2, YR1);
    addEdge(CL2, BR1);
    addEdge(CR2, GR1);
    addEdge(CR2, RR1);
    addEdge(CL3, YR2);
    addEdge(CL3, BR2);
    addEdge(CR3, GR2);
    addEdge(CR3, RR2);
    addEdge(M, TLH);
    addEdge(M, TRH);
    addEdge(M, BLH);
    addEdge(M, BRH);
    addEdge(TL, TLLH);
    addEdge(TR, TRRH);
    addEdge(BL, BLLH);
    addEdge(BL, BRRH);
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
            if(secondNode == G) route[i] = W_G;
        }
        else if(firstNode == L)
        {
            if(secondNode == S) route[i] = L_S;
        }
        else if(firstNode == G)
        {
            if(secondNode == R) route[i] = G_R;
        }
        else if(firstNode == R)
        {
            if(secondNode == B) route[i] = R_B;
        }
        else if(firstNode == B)
        {
            if(secondNode == Y) route[i] = B_Y;
        }
        else if(firstNode == Y)
        {
            if(secondNode == L) route[i] = Y_L;
        }
        else if(firstNode == M)
        {
            if(secondNode == TM) route[i] = M_TM;
            else if(secondNode == BM) route[i] = M_BM;
            else if(secondNode == CL1) route[i] = M_CL1;
            else if(secondNode == CR1) route[i] = M_CR1;
            else if(secondNode == TLH) route[i] = M_TLH;
            else if(secondNode == TRH) route[i] = M_TRH;
            else if(secondNode == BLH) route[i] = M_BLH;
            else if(secondNode == BRH) route[i] = M_BRH;
        }
        else if(firstNode == TM)
        {
            if(secondNode == M) route[i] = TM_M;
        }
        else if(firstNode == BM)
        {
            if(secondNode == M) route[i] = BM_M;
        }
        else if(firstNode == CL1)
        {
            if(secondNode == M) route[i] = CL1_M;
            else if(secondNode == CL2) route[i] = CL1_CL2;
            else if(secondNode == TL) route[i] = CL1_TL;
            else if(secondNode == BL) route[i] = CL1_BL;
        }
        else if(firstNode == CR1)
        {
            if(secondNode == M) route[i] = CR1_M;
            else if(secondNode == CR2) route[i] = CR1_CR2;
            else if(secondNode == TR) route[i] = CR1_TR;
            else if(secondNode == BR) route[i] = CR1_BR;
        }
        else if(firstNode == TL)
        {
            if(secondNode == CL1) route[i] = TL_CL1;
            else if(secondNode == TLLH) route[i] = TL_TLLH;
        }
        else if(firstNode == TR)
        {
            if(secondNode == CR1) route[i] = TR_CR1;
            else if(secondNode == TRRH) route[i] = TR_TRRH;
        }
        else if(firstNode == BL)
        {
            if(secondNode == CL1) route[i] = BL_CL1;
            else if(secondNode == BLLH) route[i] = BL_BLLH;
        }
        else if(firstNode == BR)
        {
            if(secondNode == CR1) route[i] = BR_CR1;
            else if(secondNode == BRRH) route[i] = BR_BRRH;
        }
        else if(firstNode == CL2)
        {
            if(secondNode == CL1) route[i] = CL2_CL1;
            else if(secondNode == CL3) route[i] = CL2_CL3;
            else if(secondNode == YR1) route[i] = CL2_YR1;
            else if(secondNode == BR1) route[i] = CL2_BR1;
        }
        else if(firstNode == CR2)
        {
            if(secondNode == CR1) route[i] = CR2_CR1;
            else if(secondNode == CR3) route[i] = CR2_CR3;
            else if(secondNode == GR1) route[i] = CR2_GR1;
            else if(secondNode == RR1) route[i] = CR2_RR1;
        }
        else if(firstNode == CL3)
        {
            if(secondNode == CL2) route[i] = CL3_CL2;
            else if(secondNode == YR2) route[i] = CL3_YR2;
            else if(secondNode == BR2) route[i] = CL3_BR2;
        }
        else if(firstNode == CR3)
        {
            if(secondNode == CR2) route[i] = CR3_CR2;
            else if(secondNode == GR2) route[i] = CR3_GR2;
            else if(secondNode == RR2) route[i] = CR3_RR2;
        }
        else if(firstNode == YR1)
        {
            if(secondNode == CL2) route[i] = YR1_CL2;
        }
        else if(firstNode == YR2)
        {
            if(secondNode == CL3) route[i] = YR2_CL3;
        }
        else if(firstNode == BR1)
        {
            if(secondNode == CL2) route[i] = BR1_CL2;
        }
        else if(firstNode == BR2)
        {
            if(secondNode == CL3) route[i] = BR2_CL3;
        }
        else if(firstNode == GR1)
        {
            if(secondNode == CR2) route[i] = GR1_CR2;
        }
        else if(firstNode == GR2)
        {
            if(secondNode == CR3) route[i] = GR2_CR3;
        }
        else if(firstNode == RR1)
        {
            if(secondNode == CR2) route[i] = RR1_CR2;
        }
        else if(firstNode == RR2)
        {
            if(secondNode == CR3) route[i] = RR2_CR3;
        }
        else if(firstNode == TLH)
        {
            if(secondNode == M) route[i] = TLH_M;
        }
        else if(firstNode == TRH)
        {
            if(secondNode == M) route[i] = TRH_M;
        }
        else if(firstNode == BLH)
        {
            if(secondNode == M) route[i] = BLH_M;
        }
        else if(firstNode == BRH)
        {
            if(secondNode == M) route[i] = BRH_M;
        }
        else if(firstNode == TLLH)
        {
            if(secondNode == TL) route[i] = TLLH_TL;
        }
        else if(firstNode == TRRH)
        {
            if(secondNode == TR) route[i] = TRRH_TR;
        }
        else if(firstNode == BLLH)
        {
            if(secondNode == BL) route[i] = BLLH_BL;
        }
        else if(firstNode == BRRH)
        {
            if(secondNode == BR) route[i] = BRRH_BR;
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

void standardTurn(orientation start, orientation finish, lifoRobotPosition endAlignment)
{
    int turnDifference = finish - start;
    currentDirection = finish;
    if(turnDifference == -1 || turnDifference == 3)
    {
        DEBUGPRINT("Turning left 90\n");
        leftTurn(endAlignment);
    }
    else if(turnDifference == 1 || turnDifference == -3)
    {
        DEBUGPRINT("Turning right 90\n");
        rightTurn(endAlignment);
    }
    else if(turnDifference == -2 || turnDifference == 2)
    {
        DEBUGPRINT("Reversing\n");
        reverse(currentAlignment, endAlignment);
    }
    else if(endAlignment != currentAlignment)
    {
        DEBUGPRINT("Changing Lifo Alignment\n");
        switchLifoRobotPosition(20, currentAlignment, endAlignment);
    }
    currentAlignment = endAlignment;
}

void centralTurn(orientation start, orientation finish)
{
    int turnDifference = finish - start;
    currentDirection = finish;
    robot.setMode(CONTROLLED);
    robot.setLinearAccelParams(100, 0, 0);
    if(turnDifference == -1 || turnDifference == 3)
    {
        DEBUGPRINT("Turning left 90 (Central)\n");
        robot.arc(45, -90, 0, COAST);
    }
    else if(turnDifference == 1 || turnDifference == -3)
    {
        DEBUGPRINT("Turning right 90 (Central)\n");
        robot.arc(45, 90, 0, COAST);
    }
    else if(turnDifference == -2 || turnDifference == 2)
    {
        DEBUGPRINT("Reversing (Central)\n");
        robot.arc(45, 183, 0, COAST);
    }
}

//Main network
orientation S_W(orientation dir)
{
    DEBUGPRINT("\nS_W\n");

    resetLifo();
    // lifo.setPIDparams(KP * 1.2, KI * 0.7, KD*1.5, 1);
    lifo.setPIDparams(3, 3, 120, 1);
    lifo.distance(robot.cmToTacho(35), 10, NONE);
    setLifoSlow();
    lifo.setAccelParams(150, 20, 20);
    lifo.distance(20, 3, NONE);
    lifo.lines(20, 1, NONE);

    return NORTH;
}
orientation W_G(orientation dir)
{
    DEBUGPRINT("\nW_G\n");

    //Complex turn to go from one area to another
    robot.setLinearAccelParams(100, 35, 45);
    robot.arc(45, 30, 15, NONE);
    robot.setLinearAccelParams(100, 45, 25);
    robot.arc(45, 20, 40, NONE);
    robot.setLinearAccelParams(100, 35, 35);
    robot.arcUnlim(45, 15, FORWARD, true);
    //Scan left (red room) task block while turning
    colors current = NO_COLOR;
    map<colors, int> appearances;
    while(robot.getAngle() < 40)
    {
        if((current = scanCodeBlock(leftScanner)) != NO_COLOR)
        {
            appearances[current]++;
        }
        robot.arcUnlim(45, 15, FORWARD, false);
    }
    current = analyzeFrequency(appearances, NO_COLOR);
    rooms[RED].setTask(current);
    display.resetScreen();
    display.format("%  \n")%static_cast<int>(current);

    //Get in position to scan green room task.
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


    //Lifo until right before task block (green room)
    resetLifo();
    setLifoLeftExtreme();
    lifo.distance(robot.cmToTacho(30), 5, NONE);
    setLifoLeft();
    while(rightSensor.getReflected() < 60)
        executeLifoLeftUnlim(robot.cmToTacho(30));

    //Scan (right) green room task while moving forwards on route to get in the room.
    current = NO_COLOR;
    appearances.clear();
    robot.resetPosition();
    lifo.setDoubleFollowMode("N", "N");
    lifo.unlimited(robot.cmToTacho(40), true);
    while(robot.getPosition() < 8)
    {
        if((current = scanCodeBlock(rightScanner)) != NO_COLOR)
        {
            appearances[current]++;
        }
        lifo.unlimited(robot.cmToTacho(40));
    }
    current = analyzeFrequency(appearances, NO_COLOR);
    rooms[GREEN].setTask(current);
    display.format("%  \n")%static_cast<int>(current);


    //Get to the intersection
    robot.resetPosition();
    timer t;
    t.reset();
    resetLifo();
    setLifoLeftExtreme();
    lifo.distance(robot.cmToTacho(35), 8, NONE);
    double speed = robot.getPosition()/t.secElapsed();

    setLifoLeft();
    while(rightSensor.getReflected() < 60)
        lifo.unlimited(robot.cmToTacho(35));

    robot.setMode(CONTROLLED);
    robot.setLinearAccelParams(150, speed, 0);
    robot.straight(35, 7, NONE);

    //Turn towards green room
    robot.setLinearAccelParams(100, 0, -20);
    robot.arc(45, -80, -4, NONE);
    robot.arcUnlim(20, -4, BACKWARD, true);
    while(leftSensor.getReflected() > 60)
        robot.arcUnlim(20, -4, BACKWARD);  

    //Lifo until right before green room's entrance
    resetLifo();
    setLifoLeftExtreme();
    lifo.distance(robot.cmToTacho(30), 9, NONE);
    setLifoSlow();
    setLifoLeft(true);
    lifo.setAccelParams(100, 20, 20);
    lifo.distance(20, 4, NONE);

    return SOUTH;
}
orientation G_R(orientation dir)
{
    DEBUGPRINT("\nG_R\n");
    
    //Lifo till the middle (intersection)
    resetLifo();
    setLifoRightExtreme();
    lifo.distance(robot.cmToTacho(30), 10, NONE);
    setLifoRight();
    while(leftSensor.getReflected() < 60)
        lifo.unlimited(robot.cmToTacho(35));
    lifo.setDoubleFollowMode("N", "N");
    lifo.distance(robot.cmToTacho(35), 8, NONE);


    //Lifo till right before the entrance of the red room
    resetLifo();
    setLifoRightExtreme();
    lifo.distance(robot.cmToTacho(30), 9, NONE);
    setLifoSlow();
    setLifoRight(true);
    lifo.setAccelParams(100, 20, 20);
    lifo.distance(20, 4, NONE);

    return NORTH;
}
orientation R_B(orientation dir)
{
    DEBUGPRINT("\nR_B\n");

    //Lifo exit red room till white part of intersection 
    resetLifo();
    setLifoLeftExtreme();
    lifo.distance(robot.cmToTacho(30), 10, NONE);
    setLifoLeft();
    while(rightSensor.getReflected() < 60)
        executeLifoLeftUnlim(robot.cmToTacho(30));


    //Complex turn to pass all the difficult lifo parts and get ready to cross
    robot.setMode(REGULATED);
    robot.arc(35, 30, 8.5, NONE); 
    robot.arc(35, 60, 19.5, NONE);
    robot.arcUnlim(35, 19.5, FORWARD, true);
    while(rightSensor.getReflected() < 60)
        robot.arcUnlim(35, 19.5, FORWARD);


    //Lifo slower speed to correct from turn
    robot.setMode(CONTROLLED);    
    resetLifo();
    setLifoRightExtreme();
    lifo.distance(robot.cmToTacho(30), 10, NONE);
    //Increase speed until the intersection
    setLifoRight();
    lifo.unlimited(robot.cmToTacho(45), true);
    while(leftSensor.getReflected() < 60)
        lifo.unlimited(robot.cmToTacho(45));
    //Move until right before the start/finish square
    lifo.setDoubleFollowMode("N", "N");
    lifo.distance(robot.cmToTacho(45), 8, NONE);
    setLifoRight();
    robot.resetPosition();
    timer t;
    while(robot.getPosition() < 17)
        lifo.unlimited(robot.cmToTacho(45));
    double speed = robot.getPosition() / t.secElapsed();    //Real speed calculation because of unreg/reg discrepancy
    //Pass the square (no lifo here)
    robot.setLinearAccelParams(100, speed, speed);
    robot.straight(speed, 33, NONE);    
    //Continue lifo high speed till the intersection and half way to the blue room scan area
    setLifoRightExtreme();
    lifo.distance(robot.cmToTacho(35), 6, NONE);
    setLifoRight();
    while(leftSensor.getReflected() < 60)
        lifo.unlimited(robot.cmToTacho(45));
    lifo.setDoubleFollowMode("N", "N");
    lifo.distance(robot.cmToTacho(45), 8, NONE);
    setLifoRight();
    lifo.distance(robot.cmToTacho(45), 12, NONE);

    //Slow down to correct possible mistakes and get to right before scan area
    resetLifo();
    setLifoRightExtreme();
    lifo.distance(robot.cmToTacho(35), 5, NONE);
    setLifoRight();
    while(leftSensor.getReflected() < 60)
        lifo.unlimited(robot.cmToTacho(40));

    //Continue forwards scanning left (blue room) same way as green just mirrored
    colors current = NO_COLOR;
    map<colors, int> appearances;
    robot.resetPosition();
    lifo.setDoubleFollowMode("N", "N");
    lifo.unlimited(robot.cmToTacho(40), true);
    while(robot.getPosition() < 8)
    {
        if((current = scanCodeBlock(leftScanner)) != NO_COLOR)
        {
            appearances[current]++;
        }
        lifo.unlimited(robot.cmToTacho(40));
    }
    current = analyzeFrequency(appearances, NO_COLOR);
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


    //Move to the intersection 
    robot.resetPosition();
    t.reset();
    resetLifo();
    setLifoRightExtreme();
    lifo.distance(robot.cmToTacho(35), 8, NONE);
    speed = robot.getPosition()/t.secElapsed();

    setLifoRight();
    while(leftSensor.getReflected() < 60)
        lifo.unlimited(robot.cmToTacho(35));

    robot.setMode(CONTROLLED);
    robot.setLinearAccelParams(150, speed, 0);
    robot.straight(35, 7, NONE);

    //Turn towards blue room
    robot.setLinearAccelParams(100, 0, -20);
    robot.arc(45, -80, 4, NONE);
    robot.arcUnlim(20, 4, BACKWARD, true);
    while(rightSensor.getReflected() > 60)
        robot.arcUnlim(20, 4, BACKWARD);  

    //Lifo until right before blue room's entrance
    resetLifo();
    setLifoRightExtreme();
    lifo.distance(robot.cmToTacho(30), 9, NONE);
    setLifoSlow();
    setLifoRight(true);
    lifo.setAccelParams(100, 20, 20);
    lifo.distance(20, 4, NONE);

    return SOUTH;
}
orientation B_Y(orientation dir)
{
    DEBUGPRINT("\nB_Y\n");

    //Lifo until intersection
    resetLifo();
    setLifoLeftExtreme();
    lifo.distance(robot.cmToTacho(30), 10, NONE);
    setLifoLeft();
    while(rightSensor.getReflected() < 60)
        lifo.unlimited(robot.cmToTacho(35));
    lifo.setDoubleFollowMode("N", "N");
    lifo.distance(robot.cmToTacho(35), 8, NONE);

    //Lifo until right before yellow room entrance
    resetLifo();
    setLifoLeftExtreme();
    lifo.distance(robot.cmToTacho(30), 9, NONE);
    setLifoSlow();
    setLifoLeft(true);
    lifo.setAccelParams(100, 20, 20);
    lifo.distance(20, 4, NONE);

    return NORTH;
}
orientation Y_L(orientation dir)
{
    DEBUGPRINT("\nY_L\n");

    //Exit yellow room
    resetLifo();
    setLifoRightExtreme();
    lifo.distance(robot.cmToTacho(30), 10, NONE);
    setLifoRight();
    while(leftSensor.getReflected() < 60)
        lifo.unlimited(robot.cmToTacho(30));

    
    //Special turn to pass through trouble and put line between the two sensors
    robot.setLinearAccelParams(100, 30, 30);
    robot.arc(45, 90, -8.5, COAST);

    //Correct turn mistakes with slow lifo
    resetLifo();
    // lifo.setPIDparams(KP * 1.2, KI * 0.7, KD*1.5, 1);
    lifo.setPIDparams(3, 3, 120, 1);
    lifo.distance(robot.cmToTacho(35), 5, NONE);

    //Lifo until the intersection
    timer t;
    resetLifo();
    lifo.distance(robot.cmToTacho(45), 10, NONE);
    lifo.lines(robot.cmToTacho(45), 1, NONE);

    //Lifo until right before the start/finish square
    robot.resetPosition();
    t.reset();
    lifo.distance(robot.cmToTacho(45), 22, NONE);

    double speed = robot.getPosition() / t.secElapsed(); //Speed calculation for unreg/reg behaviour
    //Complex arc turn to get into the right position before the laundry baskets
    robot.setMode(CONTROLLED);
    robot.setLinearAccelParams(150, speed, speed);
    robot.straight(speed, 6, NONE);
    robot.setLinearAccelParams(100, speed, speed);
    robot.arc(45, 90, 17.5, COAST);

    //Lifo until the intersection
    resetLifo();
    // lifo.setPIDparams(KP * 1.2, KI * 0.7, KD*1.5, 1);
    lifo.setPIDparams(3, 3, 120, 1);
    lifo.distance(robot.cmToTacho(35), 10, NONE);
    setLifoSlow();
    lifo.setAccelParams(150, 20, 20);
    lifo.distance(20, 3, NONE);
    lifo.lines(20, 1, BRAKE);

    return SOUTH;
}
orientation L_S(orientation dir)
{
    DEBUGPRINT("\nL_S\n");

    //Get to right before the start/finish square
    lifo1LineDist(CENTERED, 7, 2, 2, 3, SPECIAL_REF, NONE);
    robot.setMode(CONTROLLED);
    robot.setLinearAccelParams(100, 20, 0);
    robot.straight(45, 21, COAST);

    return NORTH;
}

//Full network standard for surprise use
orientation M_CL1(orientation dir)
{
    DEBUGPRINT("\nM_CL1\n");

    centralTurn(currentDirection, WEST);
    robot.setLinearAccelParams(100, 0, 20);
    robot.straight(45, 15, NONE);
    switchLifoRobotPosition(20, currentAlignment, CENTERED);
    lifo1LineDist(CENTERED, 20, 7, 8);

    return WEST;
}
orientation M_CR1(orientation dir)
{
    DEBUGPRINT("\nM_CR1\n");

    centralTurn(currentDirection, EAST);
    robot.setLinearAccelParams(100, 0, 20);
    robot.arc(20, 15, 20, NONE);
    robot.setLinearAccelParams(100, 20, 20);
    robot.straight(45, 12, NONE);
    currentAlignment = RIGHT_OF_LINE;
    lifo1LineDist(RIGHT_OF_LINE, 20, 7, 8);

    return EAST;
}
orientation M_TM(orientation dir)
{
    DEBUGPRINT("\nM_TM\n");

    centralTurn(currentDirection, NORTH);
    robot.setLinearAccelParams(100, 0, 20);
    robot.straight(45, 17, NONE);
    lifo1LineDist(CENTERED, 15, 5, 5);

    return NORTH;
}
orientation M_BM(orientation dir)
{
    DEBUGPRINT("\nM_BM\n");

    centralTurn(currentDirection, SOUTH);
    robot.setLinearAccelParams(100, 0, 20);
    robot.straight(45, 15, NONE);
    lifo1LineDist(CENTERED, 15, 5, 5);

    return SOUTH;
}

orientation CL1_M(orientation dir)
{
    DEBUGPRINT("\nCL1_M\n");

    standardTurn(currentDirection, EAST, CENTERED);
    lifo1LineDist(CENTERED, 15, 5, 5, 5, SPECIAL_REF, NONE);
    robot.setMode(CONTROLLED);
    robot.setLinearAccelParams(100, 20, 0);
    robot.straight(45, 21, COAST);
    
    return EAST;
}
orientation CL1_CL2(orientation dir)
{
    DEBUGPRINT("\nCL1_CL2\n");

    standardTurn(currentDirection, WEST, CENTERED);
    lifo1LineDist(CENTERED, 20, 10, 5, 5);

    return WEST;
}
orientation CL1_TL(orientation dir)
{
    DEBUGPRINT("\nCL1_TL\n");

    standardTurn(currentDirection, NORTH, CENTERED);
    lifo1LineDist(CENTERED, 25);

    return NORTH;
}
orientation CL1_BL(orientation dir)
{
    DEBUGPRINT("\nCL1_BL\n");

    standardTurn(currentDirection, SOUTH, CENTERED);
    lifo1LineDist(CENTERED, 25);

    return SOUTH;
}

orientation CR1_M(orientation dir)
{
    DEBUGPRINT("\nCR1_M\n");
    standardTurn(currentDirection, WEST, LEFT_OF_LINE);
    lifo1LineDist(LEFT_OF_LINE, 12, 2, 5, 5, SPECIAL_REF, NONE);
    robot.setMode(CONTROLLED);
    robot.setLinearAccelParams(100, 20, 0);
    robot.straight(45, 21, COAST);

    return WEST;
}
orientation CR1_CR2(orientation dir)
{
    DEBUGPRINT("\nCR1_CR2\n");

    standardTurn(currentDirection, EAST, CENTERED);
    lifo1LineDist(CENTERED, 20, 10, 5, 5);

    return EAST;
}
orientation CR1_TR(orientation dir)
{
    DEBUGPRINT("\nCR1_TR\n");

    standardTurn(currentDirection, NORTH, CENTERED);
    lifo1LineDist(CENTERED, 25);

    return NORTH;
}
orientation CR1_BR(orientation dir)
{
    DEBUGPRINT("\nCR1_BR\n");

    standardTurn(currentDirection, SOUTH, LEFT_OF_LINE);
    lifo1LineDist(LEFT_OF_LINE, 40, 10, 10, 5, NO_DETECT);

    return SOUTH;
}

orientation TM_M(orientation dir)
{
    DEBUGPRINT("\nTM_M\n");

    standardTurn(currentDirection, SOUTH, CENTERED);
    lifo1LineDist(CENTERED, 7, 2, 2, 3, SPECIAL_REF, NONE);
    robot.setMode(CONTROLLED);
    robot.setLinearAccelParams(100, 20, 0);
    robot.straight(45, 21, COAST);

    return SOUTH;
}

orientation BM_M(orientation dir)
{
    DEBUGPRINT("\nBM_M\n");

    standardTurn(currentDirection, NORTH, CENTERED);
    lifo1LineDist(CENTERED, 7, 2, 2, 3, SPECIAL_REF, NONE);
    robot.setMode(CONTROLLED);
    robot.setLinearAccelParams(100, 20, 0);
    robot.straight(45, 21, COAST);

    return NORTH;
}

orientation TL_CL1(orientation dir)
{
    DEBUGPRINT("\nTL_CL1\n");

    standardTurn(currentDirection, SOUTH, CENTERED);
    lifo1LineDist(CENTERED, 25);

    return SOUTH;
}

orientation TR_CR1(orientation dir)
{
    DEBUGPRINT("\nTR_CR1\n");

    standardTurn(currentDirection, SOUTH, CENTERED);
    lifo1LineDist(CENTERED, 25);

    return SOUTH;
}

orientation BL_CL1(orientation dir)
{
    DEBUGPRINT("\nBL_CL1\n");

    standardTurn(currentDirection, NORTH, CENTERED);
    lifo1LineDist(CENTERED, 25);

    return NORTH;
}

orientation BR_CR1(orientation dir)
{
    DEBUGPRINT("\nBR_CR1\n");

    standardTurn(currentDirection, NORTH, RIGHT_OF_LINE);
    lifo1LineDist(RIGHT_OF_LINE, 25);

    return NORTH;
}

orientation CL2_CL1(orientation dir)
{
    DEBUGPRINT("\nCL2_CL1\n");

    standardTurn(currentDirection, EAST, CENTERED);
    lifo1LineDist(CENTERED, 20, 10, 5, 5);

    return EAST;
}
orientation CL2_CL3(orientation dir)
{
    DEBUGPRINT("\nCL2_CL3\n");

    standardTurn(currentDirection, WEST, CENTERED);
    lifo1LineDist(CENTERED, 10, 0, 5, 5);

    return WEST;
}
orientation CL2_YR1(orientation dir)
{
    DEBUGPRINT("\nCL2_YR1\n");

    standardTurn(currentDirection, NORTH, CENTERED);
    lifo1LineDist(CENTERED, 10, 0, 5, 5, SPECIAL_REF);

    return NORTH;
}
orientation CL2_BR1(orientation dir)
{
    DEBUGPRINT("\nCL2_BR1\n");

    standardTurn(currentDirection, SOUTH, CENTERED);
    lifo1LineDist(CENTERED, 10, 0, 5, 5, SPECIAL_REF);

    return SOUTH;
}

orientation CR2_CR1(orientation dir)
{
    DEBUGPRINT("\nCR2_CR1\n");

    standardTurn(currentDirection, WEST, CENTERED);
    lifo1LineDist(CENTERED, 20, 10, 5, 5);

    return WEST;
}
orientation CR2_CR3(orientation dir)
{
    DEBUGPRINT("\nCR2_CR3\n");

    standardTurn(currentDirection, EAST, CENTERED);
    lifo1LineDist(CENTERED, 10, 0, 5, 5);

    return EAST;
}
orientation CR2_GR1(orientation dir)
{
    DEBUGPRINT("\nCR2_GR1\n");

    standardTurn(currentDirection, SOUTH, CENTERED);
    lifo1LineDist(CENTERED, 10, 0, 5, 5, SPECIAL_REF);

    return SOUTH;
}
orientation CR2_RR1(orientation dir)
{
    DEBUGPRINT("\nCR2_RR1\n");

    standardTurn(currentDirection, NORTH, CENTERED);
    lifo1LineDist(CENTERED, 10, 0, 5, 5, COLORED);

    return NORTH;
}

orientation CL3_CL2(orientation dir)
{
    DEBUGPRINT("\nCL3_CL2\n");

    standardTurn(currentDirection, EAST, CENTERED);
    lifo1LineDist(CENTERED, 10, 0, 5, 5);

    return EAST;
}
orientation CL3_YR2(orientation dir)
{
    DEBUGPRINT("\nCL3_YR2\n");

    standardTurn(currentDirection, NORTH, CENTERED);
    lifo1LineDist(CENTERED, 10, 0, 5, 5, SPECIAL_REF);


    return NORTH;
}
orientation CL3_BR2(orientation dir)
{
    DEBUGPRINT("\nCL3_BR2\n");

    standardTurn(currentDirection, SOUTH, CENTERED);
    lifo1LineDist(CENTERED, 10, 0, 5, 5, SPECIAL_REF);

    return SOUTH;
}

orientation CR3_CR2(orientation dir)
{
    DEBUGPRINT("\nCR3_CR2\n");

    standardTurn(currentDirection, WEST, CENTERED);
    lifo1LineDist(CENTERED, 10, 0, 5, 5);

    return WEST;
}
orientation CR3_GR2(orientation dir)
{
    DEBUGPRINT("\nCR3_GR2\n");

    standardTurn(currentDirection, SOUTH, CENTERED);
    lifo1LineDist(CENTERED, 10, 0, 5, 5, SPECIAL_REF);

    return SOUTH;
}
orientation CR3_RR2(orientation dir)
{
    DEBUGPRINT("\nCR3_RR2\n");

    standardTurn(currentDirection, NORTH, CENTERED);
    lifo1LineDist(CENTERED, 10, 0, 5, 5, COLORED);

    return NORTH;
}

orientation YR1_CL2(orientation dir)
{
    DEBUGPRINT("\nYR1_CL2\n");

    standardTurn(currentDirection, SOUTH, CENTERED);
    lifo1LineDist(CENTERED, 10, 0, 5, 5);

    return SOUTH;
}
orientation YR2_CL3(orientation dir)
{
    DEBUGPRINT("\nYR2_CL3\n");

    standardTurn(currentDirection, SOUTH, CENTERED);
    lifo1LineDist(CENTERED, 10, 0, 5, 5);

    return SOUTH;
}

orientation BR1_CL2(orientation dir)
{
    DEBUGPRINT("\nBR1_CL2\n");

    standardTurn(currentDirection, NORTH, CENTERED);
    lifo1LineDist(CENTERED, 10, 0, 5, 5);

    return NORTH;
}
orientation BR2_CL3(orientation dir)
{
    DEBUGPRINT("\nBR2_CL3\n");

    standardTurn(currentDirection, NORTH, CENTERED);
    lifo1LineDist(CENTERED, 10, 0, 5, 5);

    return NORTH;
}

orientation GR1_CR2(orientation dir)
{
    DEBUGPRINT("\nGR1_CR2\n");

    standardTurn(currentDirection, NORTH, CENTERED);
    lifo1LineDist(CENTERED, 10, 0, 5, 5);

    return NORTH;
}
orientation GR2_CR3(orientation dir)
{
    DEBUGPRINT("\nGR2_CR3\n");

    standardTurn(currentDirection, NORTH, CENTERED);
    lifo1LineDist(CENTERED, 10, 0, 5, 5);

    return NORTH;
}

orientation RR1_CR2(orientation dir)
{
    DEBUGPRINT("\nRR1_CR2\n");

    standardTurn(currentDirection, SOUTH, CENTERED);
    lifo1LineDist(CENTERED, 10, 0, 5, 5);

    return SOUTH;
}
orientation RR2_CR3(orientation dir)
{
    DEBUGPRINT("\nRR2_CR3\n");

    standardTurn(currentDirection, SOUTH, CENTERED);
    lifo1LineDist(CENTERED, 10, 0, 5, 5);

    return SOUTH;
}

orientation M_TLH(orientation dir)
{
    DEBUGPRINT("\nM_TLH\n");

    return NO;
}
orientation TLH_M(orientation dir)
{
    DEBUGPRINT("\nTLH_M\n");

    return NORTH;
}
orientation M_TRH(orientation dir)
{
    DEBUGPRINT("\nM_TRH\n");

    return NO;
}
orientation TRH_M(orientation dir)
{
    DEBUGPRINT("\nTRH_M\n");

    return NORTH;
}
orientation M_BLH(orientation dir)
{
    DEBUGPRINT("\nM_BLH\n");

    return NO;
}
orientation BLH_M(orientation dir)
{
    DEBUGPRINT("\nBLH_M\n");

    return SOUTH;
}
orientation M_BRH(orientation dir)
{
    DEBUGPRINT("\nM_BRH\n");

    return NO;
}
orientation BRH_M(orientation dir)
{
    DEBUGPRINT("\nBRH_M\n");

    return SOUTH;
}
orientation TL_TLLH(orientation dir)
{
    DEBUGPRINT("\nTL_TLLH\n");

    return NO;
}
orientation TLLH_TL(orientation dir)
{
    DEBUGPRINT("\nTLLH_TL\n");

    return SOUTH;
}
orientation TR_TRRH(orientation dir)
{
    DEBUGPRINT("\nTR_TRRH\n");

    return NO;
}
orientation TRRH_TR(orientation dir)
{
    DEBUGPRINT("\nTRRH_TR\n");

    return SOUTH;
}
orientation BL_BLLH(orientation dir)
{
    DEBUGPRINT("\nBL_BLLH\n");

    return NO;
}
orientation BLLH_BL(orientation dir)
{
    DEBUGPRINT("\nBLLH_BL\n");

    return NORTH;
}
orientation BR_BRRH(orientation dir)
{
    DEBUGPRINT("\nBR_BRRH\n");

    return NO;
}
orientation BRRH_BR(orientation dir)
{
    DEBUGPRINT("\nBRRH_BR\n");

    return NORTH;
}
