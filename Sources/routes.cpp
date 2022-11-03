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
    addEdge(S, W);      routeMapping[make_pair(S, W)] = S_W;
    addEdge(W, IR);     routeMapping[make_pair(W, IR)] = W_IR;
    addEdge(IR, G);     routeMapping[make_pair(IR, G)] = IR_G;
    addEdge(G, R);      routeMapping[make_pair(G, R)] = G_R;
    addEdge(R, IR);     routeMapping[make_pair(R, IR)] = R_IR;
    addEdge(IR, IL);    routeMapping[make_pair(IR, IL)] = IR_IL;
    addEdge(IL, B);     routeMapping[make_pair(IL, B)] = IL_B;
    addEdge(B, Y);      routeMapping[make_pair(B, Y)] = B_Y;
    addEdge(Y, IL);     routeMapping[make_pair(Y, IL)] = Y_IL;
    addEdge(IL, L);     routeMapping[make_pair(IL, L)] = IL_L;
    addEdge(L, S);      routeMapping[make_pair(L, S)] = L_S;

    //Full (surprise) network
    addEdge(M, TM);     routeMapping[make_pair(M, TM)] = M_TM;          routeMapping[make_pair(TM, M)] = TM_M;
    addEdge(M, BM);     routeMapping[make_pair(M, BM)] = M_BM;          routeMapping[make_pair(BM, M)] = BM_M;
    addEdge(M, CL1);    routeMapping[make_pair(M, CL1)] = M_CL1;        routeMapping[make_pair(CL1, M)] = CL1_M;
    addEdge(M, CR1);    routeMapping[make_pair(M, CR1)] = M_CR1;        routeMapping[make_pair(CR1, M)] = CR1_M;
    addEdge(CL1, TL);   routeMapping[make_pair(CL1, TL)] = CL1_TL;      routeMapping[make_pair(TL, CL1)] = TL_CL1;
    addEdge(CL1, BL);   routeMapping[make_pair(CL1, BL)] = CL1_BL;      routeMapping[make_pair(BL, CL1)] = BL_CL1;
    addEdge(CR1, TR);   routeMapping[make_pair(CR1, TR)] = CR1_TR;      routeMapping[make_pair(TR, CR1)] = TR_CR1;
    addEdge(CR1, BR);   routeMapping[make_pair(CR1, BR)] = CR1_BR;      routeMapping[make_pair(BR, CR1)] = BR_CR1;
    addEdge(CL1, CL2);  routeMapping[make_pair(CL1, CL2)] = CL1_CL2;    routeMapping[make_pair(CL2, CL1)] = CL2_CL1;
    addEdge(CR1, CR2);  routeMapping[make_pair(CR1, CR2)] = CR1_CR2;    routeMapping[make_pair(CR2, CR1)] = CR2_CR1;
    addEdge(CL2, CL3);  routeMapping[make_pair(CL2, CL3)] = CL2_CL3;    routeMapping[make_pair(CL3, CL2)] = CL3_CL2;
    addEdge(CR2, CR3);  routeMapping[make_pair(CR2, CR3)] = CR2_CR3;    routeMapping[make_pair(CR3, CR2)] = CR3_CR2;
    addEdge(CL2, YR1);  routeMapping[make_pair(CL2, YR1)] = CL2_YR1;    routeMapping[make_pair(YR1, CL2)] = YR1_CL2;
    addEdge(CL2, BR1);  routeMapping[make_pair(CL2, BR1)] = CL2_BR1;    routeMapping[make_pair(BR1, CL2)] = BR1_CL2;
    addEdge(CR2, GR1);  routeMapping[make_pair(CR2, GR1)] = CR2_GR1;    routeMapping[make_pair(GR1, CR2)] = GR1_CR2;
    addEdge(CR2, RR1);  routeMapping[make_pair(CR2, RR1)] = CR2_RR1;    routeMapping[make_pair(RR1, CR2)] = RR1_CR2;
    addEdge(CL3, YR2);  routeMapping[make_pair(CL3, YR2)] = CL3_YR2;    routeMapping[make_pair(YR2, CL3)] = YR2_CL3;
    addEdge(CL3, BR2);  routeMapping[make_pair(CL3, BR2)] = CL3_BR2;    routeMapping[make_pair(BR2, CL3)] = BR2_CL3;
    addEdge(CR3, GR2);  routeMapping[make_pair(CR3, GR2)] = CR3_GR2;    routeMapping[make_pair(GR2, CR3)] = GR2_CR3;
    addEdge(CR3, RR2);  routeMapping[make_pair(CR3, RR2)] = CR3_RR2;    routeMapping[make_pair(RR2, CR3)] = RR2_CR3;
    addEdge(M, TLH);    routeMapping[make_pair(M, TLH)] = M_TLH;        routeMapping[make_pair(TLH, M)] = TLH_M;
    addEdge(M, TRH);    routeMapping[make_pair(M, TRH)] = M_TRH;        routeMapping[make_pair(TRH, M)] = TRH_M;
    addEdge(M, BLH);    routeMapping[make_pair(M, BLH)] = M_BLH;        routeMapping[make_pair(BLH, M)] = BLH_M;
    addEdge(M, BRH);    routeMapping[make_pair(M, BRH)] = M_BRH;        routeMapping[make_pair(BRH, M)] = BRH_M;
    addEdge(TL, TLLH);  routeMapping[make_pair(TL, TLLH)] = TL_TLLH;    routeMapping[make_pair(TLLH, TL)] = TLLH_TL;
    addEdge(TR, TRRH);  routeMapping[make_pair(TR, TRRH)] = TR_TRRH;    routeMapping[make_pair(TRRH, TR)] = TRRH_TR;
    addEdge(BL, BLLH);  routeMapping[make_pair(BL, BLLH)] = BL_BLLH;    routeMapping[make_pair(BLLH, BL)] = BLLH_BL;
    addEdge(BR, BRRH);  routeMapping[make_pair(BR, BRRH)] = BR_BRRH;    routeMapping[make_pair(BRRH, BR)] = BRRH_BR;

    //Secondary routing (faster, but may require higher maintenance)
    //Vertical
    addEdge(BM, TM);    routeMapping[make_pair(BM, TM)] = BM_TM;        routeMapping[make_pair(TM, BM)] = TM_BM;
    addEdge(BL, TL);    routeMapping[make_pair(BL, TL)] = BL_TL;        routeMapping[make_pair(TL, BL)] = TL_BL;
    addEdge(BR, TR);    routeMapping[make_pair(BR, TR)] = BR_TR;        routeMapping[make_pair(TR, BR)] = TR_BR;
    addEdge(BR1, YR1);  routeMapping[make_pair(BR1, YR1)] = BR1_YR1;    routeMapping[make_pair(YR1, BR1)] = YR1_BR1;
    addEdge(BR2, YR2);  routeMapping[make_pair(BR2, YR2)] = BR2_YR2;    routeMapping[make_pair(YR2, BR2)] = YR2_BR2;
    addEdge(GR1, RR1);  routeMapping[make_pair(GR1, RR1)] = GR1_RR1;    routeMapping[make_pair(RR1, GR1)] = RR1_GR1;
    addEdge(GR2, RR2);  routeMapping[make_pair(GR2, RR2)] = GR2_RR2;    routeMapping[make_pair(RR2, GR2)] = RR2_GR2;
    //Double skips
    addEdge(CL3, CL1);  routeMapping[make_pair(CL3, CL1)] = CL3_CL1;    routeMapping[make_pair(CL1, CL3)] = CL1_CL3;
    addEdge(CL2, M);    routeMapping[make_pair(CL2, M)] = CL2_M;        routeMapping[make_pair(M, CL2)] = M_CL2;
    addEdge(CL1, CR1);  routeMapping[make_pair(CL1, CR1)] = CL1_CR1;    routeMapping[make_pair(CR1, CL1)] = CR1_CL1;
    addEdge(M, CR2);    routeMapping[make_pair(M, CR2)] = M_CR2;        routeMapping[make_pair(CR2, M)] = CR2_M;
    addEdge(CR1, CR3);  routeMapping[make_pair(CR1, CR3)] = CR1_CR3;    routeMapping[make_pair(CR3, CR1)] = CR3_CR1;
    //Triple skips
    addEdge(CL3, M);    routeMapping[make_pair(CL3, M)] = CL3_M;        routeMapping[make_pair(M, CL3)] = M_CL3;
    addEdge(CL2, CR1);  routeMapping[make_pair(CL2, CR1)] = CL2_CR1;    routeMapping[make_pair(CR1, CL2)] = CR1_CL2;
    addEdge(CL1, CR2);  routeMapping[make_pair(CL1, CR2)] = CL1_CR2;    routeMapping[make_pair(CR2, CL1)] = CR2_CL1;
    addEdge(M, CR3);    routeMapping[make_pair(M, CR3)] = M_CR3;        routeMapping[make_pair(CR3, M)] = CR3_M;
    //Quadruple skips
    addEdge(CL3, CR1);  routeMapping[make_pair(CL3, CR1)] = CL3_CR1;    routeMapping[make_pair(CR1, CL3)] = CR1_CL3;
    addEdge(CL2, CR2);  routeMapping[make_pair(CL2, CR2)] = CL2_CR2;    routeMapping[make_pair(CR2, CL2)] = CR2_CL2;
    addEdge(CL1, CR3);  routeMapping[make_pair(CL1, CR3)] = CL1_CR3;    routeMapping[make_pair(CR3, CL1)] = CR3_CL1;
    //Quintuple skips   
    addEdge(CL3, CR2);  routeMapping[make_pair(CL3, CR2)] = CL3_CR2;    routeMapping[make_pair(CR2, CL3)] = CR2_CL3;
    addEdge(CL2, CR3);  routeMapping[make_pair(CL2, CR3)] = CL2_CR3;    routeMapping[make_pair(CR3, CL2)] = CR3_CL2;
    //Sixtuple skips
    addEdge(CL3, CR3);  routeMapping[make_pair(CL3, CR3)] = CL3_CR3;    routeMapping[make_pair(CR3, CL3)] = CR3_CL3;
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
        route[i] = routeMapping[make_pair(static_cast<matPos>(firstNode), static_cast<matPos>(secondNode))];
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

    setLifo("SL", "SR");
    lifoUnregExtreme.distance(30, 8, NONE);
    lifoUnregNormal.lines(40, 1, NONE, 5);

    return NORTH;
}
orientation W_IR(orientation dir)
{
    DEBUGPRINT("\nW_IR\n");

    //Special turn to go from TR to CR2(nearly) and scan red room task
    robot.setMode(CONTROLLED);
    robot.setLinearAccelParams(100, 40, 45);
    robot.arc(45, 30, 15, NONE);
    robot.setLinearAccelParams(100, 45, 45);
    robot.arc(45, 20, 40, NONE);
    robot.setLinearAccelParams(100, 45, 35);

    stopScanning = false;
    scanner = &leftScanner;
    act_tsk(ROOM_TASK_SCAN_TASK);
    tslp_tsk(1);
    robot.arc(45, 45, 15, NONE);
    stopScanning = true;

    robot.setLinearAccelParams(100, 35, 45);
    robot.straight(45, 5, NONE);

    rooms[RED].setTask(scannedValue);
    display.resetScreen();
    display.format("%  \n")%static_cast<int>(scannedValue);
    
    //Straight move but uses lines for location help
    leftSensor.resetFiltering();
    robot.setLinearAccelParams(100, 45, 45);
    robot.straightUnlim(45, true);
    do
    {
        leftSensor.getReflected();
        robot.straightUnlim(45);
    } while (!leftSensor.getLineDetected());
    robot.straight(45, 7, NONE);
    leftSensor.resetFiltering();
    robot.straightUnlim(45, true);
    do
    {
        leftSensor.getReflected();
        robot.straightUnlim(45);
    } while (!leftSensor.getLineDetected());
    robot.setLinearAccelParams(100, 45, 0);
    robot.straight(45, 8.5, COAST);

    //Turn wide back and limit with line
    robot.setLinearAccelParams(100, 0, -25);
    robot.arc(45, -85, 3.5, NONE);
    robot.setLinearAccelParams(100, -25, -25);
    robot.arcUnlim(25, 3.5, BACKWARD, true);
    while(leftSensor.getReflected() < 50 && abs(robot.getAngle()) < 10)
        robot.arcUnlim(25, 3.5, BACKWARD, false);
    robot.stop(COAST);
 
    return EAST;
}
orientation IR_G(orientation dir)
{
    DEBUGPRINT("\nIR_G\n");

    stopScanning = false;
    scanner = &rightScanner;
    act_tsk(ROOM_TASK_SCAN_TASK);
    tslp_tsk(1);

    setLifo("SL", "70");
    lifoUnregNormal.lines(30, 1, NONE, 4, 8.5, true);
    stopScanning = true;
    lifoUnregNormal.distance(30, 5, NONE);
    lifoControlled.lines(30, 1, NONE, 5);

    rooms[GREEN].setTask(scannedValue);
    display.format("%  \n")%static_cast<int>(scannedValue);

    robot.setLinearAccelParams(100, 30, 0);
    robot.straight(30, 8, COAST);

    robot.setLinearAccelParams(100, 0, -25);
    robot.arc(50, -85, -5, NONE);
    robot.setLinearAccelParams(100, -25, -25);
    robot.arcUnlim(25, -5, BACKWARD, true);
    while(leftSensor.getReflected() > 80 && abs(robot.getAngle()) < 10)
        robot.arcUnlim(25, -5, BACKWARD, false);
    robot.stop(COAST);

    setLifo("SL", "70");
    lifoUnregNormal.distance(30, 12, NONE);

    return SOUTH;
}
orientation G_R(orientation dir)
{
    DEBUGPRINT("\nG_R\n");
    
    setLifo("70", "SR");
    lifoUnregExtreme.distance(30, 5, NONE);
    lifoUnregExtreme.distance(40, 5, NONE);
    lifoUnregNormal.lines(40, 1, NONE, 5, 8.5, true);
    lifoUnregNormal.distance(30, 12, NONE);

    return NORTH;
}
orientation R_IR(orientation dir)
{
    DEBUGPRINT("\nR_IR\n");

    setLifo("SL", "70");
    lifoUnregExtreme.distance(30, 5, NONE);
    lifoUnregExtreme.distance(40, 5, NONE);
    lifoUnregNormal.lines(30, 1, NONE, 5);

    robot.setMode(CONTROLLED);
    robot.setLinearAccelParams(100, 30, 30);
    robot.arc(45, 40, 8.5, NONE);
    robot.setLinearAccelParams(100, 40, 25);
    robot.arc(45, 55, 18, NONE);
    robot.arcUnlim(25, 18, FORWARD, true);
    while(rightSensor.getReflected() < 80 && abs(robot.getAngle()) < 10)
        robot.arcUnlim(25, 18, FORWARD, false);

    return WEST;
}
orientation IR_IL(orientation dir)
{
    DEBUGPRINT("\nIR_IL\n");

    setLifo("70", "SR");
    lifoUnregExtreme.distance(30, 5, NONE);
    lifoUnregExtreme.distance(40, 5, NONE);
    lifoUnregNormal.lines(40, 1, NONE, 5, 8.5, true);
    lifoUnregNormal.distance(40, 17, NONE);

    robot.setMode(CONTROLLED);
    robot.setLinearAccelParams(100, 40, 30);
    robot.straight(50, 34, NONE);

    lifoUnregExtreme.distance(30, 5, NONE);
    lifoUnregExtreme.distance(40, 5, NONE);
    lifoUnregNormal.lines(40, 1, NONE, 5, 8.5, true);
    
    return WEST;

}
orientation IL_B(orientation dir)
{
    DEBUGPRINT("\nIL_B\n");

    stopScanning = false;
    scanner = &leftScanner;
    act_tsk(ROOM_TASK_SCAN_TASK);
    tslp_tsk(1);

    setLifo("70", "SR");
    lifoUnregNormal.lines(40, 1, NONE, 5, 8.5, true);
    stopScanning = true;
    lifoUnregNormal.distance(30, 5, NONE);
    lifoControlled.lines(30, 1, NONE, 5);

    rooms[BLUE].setTask(scannedValue);
    display.format("%  \n")%static_cast<int>(scannedValue);

    robot.setLinearAccelParams(100, 30, 0);
    robot.straight(30, 8, COAST);

    robot.setLinearAccelParams(100, 0, -25);
    robot.arc(50, -85, 5, NONE);
    robot.setLinearAccelParams(100, -25, -25);
    robot.arcUnlim(25, 5, BACKWARD, true);
    while(rightSensor.getReflected() > 80 && abs(robot.getAngle()) < 10)
        robot.arcUnlim(25, 5, BACKWARD, false);
    robot.stop(COAST);

    setLifo("70", "SR");
    lifoUnregNormal.distance(30, 12, NONE);

    inferYellowRoomTask();

    return SOUTH;
}
orientation B_Y(orientation dir)
{
    DEBUGPRINT("\nB_Y\n");

    setLifo("SL", "70");
    lifoUnregExtreme.distance(30, 5, NONE);
    lifoUnregExtreme.distance(40, 5, NONE);
    lifoUnregNormal.lines(40, 1, NONE, 5, 8.5, true);
    lifoUnregNormal.distance(30, 12, NONE);

    return NORTH;
}
orientation Y_IL(orientation dir)
{
    DEBUGPRINT("\nY_IL\n");

    setLifo("70", "SR");
    lifoUnregExtreme.distance(30, 5, NONE);
    lifoUnregExtreme.distance(40, 5, NONE);
    lifoUnregNormal.lines(30, 1, NONE, 5);

    robot.setMode(CONTROLLED);
    robot.setLinearAccelParams(100, 30, 30);
    robot.arc(45, 90, -8.5, NONE);

    return EAST;
}
orientation IL_L(orientation dir)
{
    DEBUGPRINT("\nIL_L\n");

    setLifo("SL", "SR");
    lifoUnregExtreme.distance(30, 7, NONE);
    lifoUnregNormal.lines(40, 2, NONE, 20, 2.5, false);

    robot.setMode(CONTROLLED);
    robot.setLinearAccelParams(100, 40, 30);
    robot.straight(40, 5, NONE);

    robot.setLinearAccelParams(100, 30, 40);
    robot.arc(45, 90, 16, NONE);

    lifoUnregExtreme.distance(30, 5, NONE);
    lifoUnregNormal.distance(40, 5, NONE);
    lifoControlled.lines(30, 1, COAST, 5);

    return SOUTH;
}
orientation L_S(orientation dir)
{
    DEBUGPRINT("\nL_S\n");
    
    lifoUnregExtreme.distance(30, 5, NONE);
    lifoUnregNormal.lines(40, 1, NONE, 5);

    robot.setMode(CONTROLLED);
    robot.setLinearAccelParams(100, 40, 0);
    robot.straight(45, 21, COAST);

    return NORTH;
}

//Full network standard for surprise use
orientation M_CL1(orientation dir)
{
    DEBUGPRINT("\nM_CL1\n");

    // centralTurn(currentDirection, WEST);
    // robot.setLinearAccelParams(100, 0, 20);
    // robot.straight(45, 15, NONE);
    // switchLifoRobotPosition(20, currentAlignment, CENTERED);
    // lifo1LineDist(CENTERED, 20, 7, 8);

    return WEST;
}
orientation M_CR1(orientation dir)
{
    DEBUGPRINT("\nM_CR1\n");

    // centralTurn(currentDirection, EAST);
    // robot.setLinearAccelParams(100, 0, 20);
    // robot.arc(20, 15, 20, NONE);
    // robot.setLinearAccelParams(100, 20, 20);
    // robot.straight(45, 12, NONE);
    // currentAlignment = RIGHT_OF_LINE;
    // lifo1LineDist(RIGHT_OF_LINE, 20, 7, 8);

    return EAST;
}
orientation M_TM(orientation dir)
{
    DEBUGPRINT("\nM_TM\n");

    // centralTurn(currentDirection, NORTH);
    // robot.setLinearAccelParams(100, 0, 20);
    // robot.straight(45, 17, NONE);
    // lifo1LineDist(CENTERED, 15, 5, 5);

    return NORTH;
}
orientation M_BM(orientation dir)
{
    DEBUGPRINT("\nM_BM\n");

    // centralTurn(currentDirection, SOUTH);
    // robot.setLinearAccelParams(100, 0, 20);
    // robot.straight(45, 15, NONE);
    // lifo1LineDist(CENTERED, 15, 5, 5);

    return SOUTH;
}

orientation CL1_M(orientation dir)
{
    DEBUGPRINT("\nCL1_M\n");

    // standardTurn(currentDirection, EAST, CENTERED);
    // lifo1LineDist(CENTERED, 15, 5, 5, 5, SPECIAL_REF, NONE);
    // robot.setMode(CONTROLLED);
    // robot.setLinearAccelParams(100, 20, 0);
    // robot.straight(45, 21, COAST);
    
    return EAST;
}
orientation CL1_CL2(orientation dir)
{
    DEBUGPRINT("\nCL1_CL2\n");

    // standardTurn(currentDirection, WEST, CENTERED);
    // lifo1LineDist(CENTERED, 20, 10, 5, 5);

    return WEST;
}
orientation CL1_TL(orientation dir)
{
    DEBUGPRINT("\nCL1_TL\n");

    // standardTurn(currentDirection, NORTH, CENTERED);
    // lifo1LineDist(CENTERED, 25);

    return NORTH;
}
orientation CL1_BL(orientation dir)
{
    DEBUGPRINT("\nCL1_BL\n");

    // standardTurn(currentDirection, SOUTH, CENTERED);
    // lifo1LineDist(CENTERED, 25);

    return SOUTH;
}

orientation CR1_M(orientation dir)
{
    DEBUGPRINT("\nCR1_M\n");

    // standardTurn(currentDirection, WEST, LEFT_OF_LINE);
    // lifo1LineDist(LEFT_OF_LINE, 12, 2, 5, 5, SPECIAL_REF, NONE);
    // robot.setMode(CONTROLLED);
    // robot.setLinearAccelParams(100, 20, 0);
    // robot.straight(45, 21, COAST);

    return WEST;
}
orientation CR1_CR2(orientation dir)
{
    DEBUGPRINT("\nCR1_CR2\n");

    // standardTurn(currentDirection, EAST, CENTERED);
    // lifo1LineDist(CENTERED, 20, 10, 5, 5);

    return EAST;
}
orientation CR1_TR(orientation dir)
{
    DEBUGPRINT("\nCR1_TR\n");

    // standardTurn(currentDirection, NORTH, CENTERED);
    // lifo1LineDist(CENTERED, 25);

    return NORTH;
}
orientation CR1_BR(orientation dir)
{
    DEBUGPRINT("\nCR1_BR\n");

    // standardTurn(currentDirection, SOUTH, LEFT_OF_LINE);
    // lifo1LineDist(LEFT_OF_LINE, 40, 10, 10, 5, NO_DETECT);

    return SOUTH;
}

orientation TM_M(orientation dir)
{
    DEBUGPRINT("\nTM_M\n");

    // standardTurn(currentDirection, SOUTH, CENTERED);
    // lifo1LineDist(CENTERED, 7, 2, 2, 3, SPECIAL_REF, NONE);
    // robot.setMode(CONTROLLED);
    // robot.setLinearAccelParams(100, 20, 0);
    // robot.straight(45, 21, COAST);

    return SOUTH;
}

orientation BM_M(orientation dir)
{
    DEBUGPRINT("\nBM_M\n");

    // standardTurn(currentDirection, NORTH, CENTERED);
    // lifo1LineDist(CENTERED, 7, 2, 2, 3, SPECIAL_REF, NONE);
    // robot.setMode(CONTROLLED);
    // robot.setLinearAccelParams(100, 20, 0);
    // robot.straight(45, 21, COAST);

    return NORTH;
}

orientation TL_CL1(orientation dir)
{
    DEBUGPRINT("\nTL_CL1\n");

    // standardTurn(currentDirection, SOUTH, CENTERED);
    // lifo1LineDist(CENTERED, 25);

    return SOUTH;
}

orientation TR_CR1(orientation dir)
{
    DEBUGPRINT("\nTR_CR1\n");

    // standardTurn(currentDirection, SOUTH, CENTERED);
    // lifo1LineDist(CENTERED, 25);

    return SOUTH;
}

orientation BL_CL1(orientation dir)
{
    DEBUGPRINT("\nBL_CL1\n");

    // standardTurn(currentDirection, NORTH, CENTERED);
    // lifo1LineDist(CENTERED, 25);

    return NORTH;
}

orientation BR_CR1(orientation dir)
{
    DEBUGPRINT("\nBR_CR1\n");

    // standardTurn(currentDirection, NORTH, RIGHT_OF_LINE);
    // lifo1LineDist(RIGHT_OF_LINE, 25);

    return NORTH;
}

orientation CL2_CL1(orientation dir)
{
    DEBUGPRINT("\nCL2_CL1\n");

    // standardTurn(currentDirection, EAST, CENTERED);
    // lifo1LineDist(CENTERED, 20, 10, 5, 5);

    return EAST;
}
orientation CL2_CL3(orientation dir)
{
    DEBUGPRINT("\nCL2_CL3\n");

    // standardTurn(currentDirection, WEST, CENTERED);
    // lifo1LineDist(CENTERED, 10, 0, 5, 5);

    return WEST;
}
orientation CL2_YR1(orientation dir)
{
    DEBUGPRINT("\nCL2_YR1\n");

    // standardTurn(currentDirection, NORTH, CENTERED);
    // lifo1LineDist(CENTERED, 10, 0, 5, 5, SPECIAL_REF);

    return NORTH;
}
orientation CL2_BR1(orientation dir)
{
    DEBUGPRINT("\nCL2_BR1\n");

    // standardTurn(currentDirection, SOUTH, CENTERED);
    // lifo1LineDist(CENTERED, 10, 0, 5, 5, SPECIAL_REF);

    return SOUTH;
}

orientation CR2_CR1(orientation dir)
{
    DEBUGPRINT("\nCR2_CR1\n");

    // standardTurn(currentDirection, WEST, CENTERED);
    // lifo1LineDist(CENTERED, 20, 10, 5, 5);

    return WEST;
}
orientation CR2_CR3(orientation dir)
{
    DEBUGPRINT("\nCR2_CR3\n");

    // standardTurn(currentDirection, EAST, CENTERED);
    // lifo1LineDist(CENTERED, 10, 0, 5, 5);

    return EAST;
}
orientation CR2_GR1(orientation dir)
{
    DEBUGPRINT("\nCR2_GR1\n");

    // standardTurn(currentDirection, SOUTH, CENTERED);
    // lifo1LineDist(CENTERED, 10, 0, 5, 5, SPECIAL_REF);

    return SOUTH;
}
orientation CR2_RR1(orientation dir)
{
    DEBUGPRINT("\nCR2_RR1\n");

    // standardTurn(currentDirection, NORTH, CENTERED);
    // lifo1LineDist(CENTERED, 10, 0, 5, 5, COLORED);

    return NORTH;
}

orientation CL3_CL2(orientation dir)
{
    DEBUGPRINT("\nCL3_CL2\n");

    // standardTurn(currentDirection, EAST, CENTERED);
    // lifo1LineDist(CENTERED, 10, 0, 5, 5);

    return EAST;
}
orientation CL3_YR2(orientation dir)
{
    DEBUGPRINT("\nCL3_YR2\n");

    // standardTurn(currentDirection, NORTH, CENTERED);
    // lifo1LineDist(CENTERED, 10, 0, 5, 5, SPECIAL_REF);

    return NORTH;
}
orientation CL3_BR2(orientation dir)
{
    DEBUGPRINT("\nCL3_BR2\n");

    // standardTurn(currentDirection, SOUTH, CENTERED);
    // lifo1LineDist(CENTERED, 10, 0, 5, 5, SPECIAL_REF);

    return SOUTH;
}

orientation CR3_CR2(orientation dir)
{
    DEBUGPRINT("\nCR3_CR2\n");

    // standardTurn(currentDirection, WEST, CENTERED);
    // lifo1LineDist(CENTERED, 10, 0, 5, 5);

    return WEST;
}
orientation CR3_GR2(orientation dir)
{
    DEBUGPRINT("\nCR3_GR2\n");

    // standardTurn(currentDirection, SOUTH, CENTERED);
    // lifo1LineDist(CENTERED, 10, 0, 5, 5, SPECIAL_REF);

    return SOUTH;
}
orientation CR3_RR2(orientation dir)
{
    DEBUGPRINT("\nCR3_RR2\n");

    // standardTurn(currentDirection, NORTH, CENTERED);
    // lifo1LineDist(CENTERED, 10, 0, 5, 5, COLORED);

    return NORTH;
}

orientation YR1_CL2(orientation dir)
{
    DEBUGPRINT("\nYR1_CL2\n");

    // standardTurn(currentDirection, SOUTH, CENTERED);
    // lifo1LineDist(CENTERED, 10, 0, 5, 5);

    return SOUTH;
}
orientation YR2_CL3(orientation dir)
{
    DEBUGPRINT("\nYR2_CL3\n");

    // standardTurn(currentDirection, SOUTH, CENTERED);
    // lifo1LineDist(CENTERED, 10, 0, 5, 5);

    return SOUTH;
}

orientation BR1_CL2(orientation dir)
{
    DEBUGPRINT("\nBR1_CL2\n");

    // standardTurn(currentDirection, NORTH, CENTERED);
    // lifo1LineDist(CENTERED, 10, 0, 5, 5);

    return NORTH;
}
orientation BR2_CL3(orientation dir)
{
    DEBUGPRINT("\nBR2_CL3\n");

    // standardTurn(currentDirection, NORTH, CENTERED);
    // lifo1LineDist(CENTERED, 10, 0, 5, 5);

    return NORTH;
}

orientation GR1_CR2(orientation dir)
{
    DEBUGPRINT("\nGR1_CR2\n");

    // standardTurn(currentDirection, NORTH, CENTERED);
    // lifo1LineDist(CENTERED, 10, 0, 5, 5);

    return NORTH;
}
orientation GR2_CR3(orientation dir)
{
    DEBUGPRINT("\nGR2_CR3\n");

    // standardTurn(currentDirection, NORTH, CENTERED);
    // lifo1LineDist(CENTERED, 10, 0, 5, 5);

    return NORTH;
}

orientation RR1_CR2(orientation dir)
{
    DEBUGPRINT("\nRR1_CR2\n");

    // standardTurn(currentDirection, SOUTH, CENTERED);
    // lifo1LineDist(CENTERED, 10, 0, 5, 5);

    return SOUTH;
}
orientation RR2_CR3(orientation dir)
{
    DEBUGPRINT("\nRR2_CR3\n");

    // standardTurn(currentDirection, SOUTH, CENTERED);
    // lifo1LineDist(CENTERED, 10, 0, 5, 5);

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


//Secondary routing (faster, but may require higher maintenance)
//Vertical
orientation BM_TM(orientation dir)
{
    DEBUGPRINT("\nBM_TM\n");

    return NORTH;
}
orientation TM_BM(orientation dir)
{
    DEBUGPRINT("\nTM_BM\n");

    return SOUTH;
}

orientation BL_TL(orientation dir)
{
    DEBUGPRINT("\nBL_TL\n");

    return NORTH;
}
orientation TL_BL(orientation dir)
{
    DEBUGPRINT("\nTL_BL\n");

    return SOUTH;
}

orientation BR_TR(orientation dir)
{
    DEBUGPRINT("\nBR_TR\n");

    return NORTH;
}
orientation TR_BR(orientation dir)
{
    DEBUGPRINT("\nTR_BR\n");

    return SOUTH;
}

orientation BR1_YR1(orientation dir)
{
    DEBUGPRINT("\nBR1_YR1\n");

    return NORTH;
}
orientation YR1_BR1(orientation dir)
{
    DEBUGPRINT("\nYR1_BR1\n");

    return SOUTH;
}
orientation BR2_YR2(orientation dir)
{
    DEBUGPRINT("\nBR2_YR2\n");

    return NORTH;
}
orientation YR2_BR2(orientation dir)
{
    DEBUGPRINT("\nYR2_BR2\n");

    return SOUTH;
}

orientation GR1_RR1(orientation dir)
{
    DEBUGPRINT("\nGR1_RR1\n");

    return NORTH;
}
orientation RR1_GR1(orientation dir)
{
    DEBUGPRINT("\nRR1_GR1\n");

    return SOUTH;
}
orientation GR2_RR2(orientation dir)
{
    DEBUGPRINT("\nGR2_RR2\n");

    return NORTH;
}
orientation RR2_GR2(orientation dir)
{
    DEBUGPRINT("\nRR2_GR2\n");

    return SOUTH;
}

//Double skips
orientation CL3_CL1(orientation dir)
{
    DEBUGPRINT("\nCL3_CL1\n");

    return EAST;
}
orientation CL1_CL3(orientation dir)
{
    DEBUGPRINT("\nCL1_CL3\n");

    return WEST;
}
orientation CL2_M(orientation dir)
{
    DEBUGPRINT("\nCL2_M\n");

    return EAST;
}
orientation M_CL2(orientation dir)
{
    DEBUGPRINT("\nM_CL2\n");

    return WEST;
}
orientation CL1_CR1(orientation dir)
{
    DEBUGPRINT("\nCL1_CR1\n");

    return EAST;
}
orientation CR1_CL1(orientation dir)
{
    DEBUGPRINT("\nCR1_CL1\n");

    return WEST;
}
orientation M_CR2(orientation dir)
{
    DEBUGPRINT("\nM_CR2\n");

    return EAST;
}
orientation CR2_M(orientation dir)
{
    DEBUGPRINT("\nCR2_M\n");

    return WEST;
}
orientation CR1_CR3(orientation dir)
{
    DEBUGPRINT("\nCR1_CR3\n");

    return EAST;
}
orientation CR3_CR1(orientation dir)
{
    DEBUGPRINT("\nCR3_CR1\n");

    return WEST;
}

//Triple skips
orientation CL3_M(orientation dir)
{
    DEBUGPRINT("\nCL3_M\n");

    return EAST;
}
orientation M_CL3(orientation dir)
{
    DEBUGPRINT("\nM_CL3\n");

    return WEST;
}
orientation CL2_CR1(orientation dir)
{
    DEBUGPRINT("\nCL2_CR1\n");

    return EAST;
}
orientation CR1_CL2(orientation dir)
{
    DEBUGPRINT("\nCR1_CL2\n");

    return WEST;
}
orientation CL1_CR2(orientation dir)
{
    DEBUGPRINT("\nCL1_CR2\n");

    return EAST;
}
orientation CR2_CL1(orientation dir)
{
    DEBUGPRINT("\nCR2_CL1\n");

    return WEST;
}
orientation M_CR3(orientation dir)
{
    DEBUGPRINT("\nM_CR3\n");

    return EAST;
}
orientation CR3_M(orientation dir)
{
    DEBUGPRINT("\nCR3_M\n");

    return WEST;
}

//Quadruple skips
orientation CL3_CR1(orientation dir)
{
    DEBUGPRINT("\nCL3_CR1\n");

    return EAST;
}
orientation CR1_CL3(orientation dir)
{
    DEBUGPRINT("\nCR1_CL3\n");

    return WEST;
}
orientation CL2_CR2(orientation dir)
{
    DEBUGPRINT("\nCL2_CR2\n");

    return EAST;
}
orientation CR2_CL2(orientation dir)
{
    DEBUGPRINT("\nCR2_CL2\n");

    return WEST;
}
orientation CL1_CR3(orientation dir)
{
    DEBUGPRINT("\nCL1_CR3\n");

    return EAST;
}
orientation CR3_CL1(orientation dir)
{
    DEBUGPRINT("\nCR3_CL1\n");

    return WEST;
}

//Quintuple skips
orientation CL3_CR2(orientation dir)
{
    DEBUGPRINT("\nCL3_CR2\n");

    return EAST;
}
orientation CR2_CL3(orientation dir)
{
    DEBUGPRINT("\nCR2_CL3\n");

    return WEST;
}
orientation CL2_CR3(orientation dir)
{
    DEBUGPRINT("\nCL2_CR3\n");

    return EAST;
}
orientation CR3_CL2(orientation dir)
{
    DEBUGPRINT("\nCR3_CL2\n");

    return WEST;
}

//Sixtuple skips
orientation CL3_CR3(orientation dir)
{
    DEBUGPRINT("\nCL3_CR3\n");

    return EAST;
}
orientation CR3_CL3(orientation dir)
{
    DEBUGPRINT("\nCR3_CL3\n");

    return WEST;
}
