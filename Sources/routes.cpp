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
    ADD_EDGE(M, TM);
    ADD_EDGE(M, BM);
    ADD_EDGE(M, CL1);
    ADD_EDGE(M, CR1);
    ADD_EDGE(CL1, TL);
    ADD_EDGE(CL1, BL);
    ADD_EDGE(CR1, TR);
    ADD_EDGE(CR1, BR);
    ADD_EDGE(CL1, CL2);
    ADD_EDGE(CR1, CR2);
    ADD_EDGE(CL2, CL3);
    ADD_EDGE(CR2, CR3);
    ADD_EDGE(CL2, YR1);
    ADD_EDGE(CL2, BR1);
    ADD_EDGE(CR2, GR1);
    ADD_EDGE(CR2, RR1);
    ADD_EDGE(CL3, YR2);
    ADD_EDGE(CL3, BR2);
    ADD_EDGE(CR3, GR2);
    ADD_EDGE(CR3, RR2);
    ADD_EDGE(M, TLH);
    ADD_EDGE(M, TRH);
    ADD_EDGE(M, BLH);
    ADD_EDGE(M, BRH);
    ADD_EDGE(TL, TLLH);
    ADD_EDGE(TR, TRRH);
    ADD_EDGE(BL, BLLH);
    ADD_EDGE(BR, BRRH);

    // //Secondary routing (faster, but may require higher maintenance)
    // //Vertical
    // addEdge(BM, TM);    routeMapping[make_pair(BM, TM)] = BM_TM;        routeMapping[make_pair(TM, BM)] = TM_BM;
    // addEdge(BL, TL);    routeMapping[make_pair(BL, TL)] = BL_TL;        routeMapping[make_pair(TL, BL)] = TL_BL;
    // addEdge(BR, TR);    routeMapping[make_pair(BR, TR)] = BR_TR;        routeMapping[make_pair(TR, BR)] = TR_BR;
    ADD_EDGE(BR1, YR1);
    ADD_EDGE(BR2, YR2);
    ADD_EDGE(GR1, RR1);
    ADD_EDGE(GR2, RR2);
    // //Double skips
    ADD_EDGE(CL3, CL1);
    // addEdge(CL2, M);    routeMapping[make_pair(CL2, M)] = CL2_M;        routeMapping[make_pair(M, CL2)] = M_CL2;
    // addEdge(CL1, CR1);  routeMapping[make_pair(CL1, CR1)] = CL1_CR1;    routeMapping[make_pair(CR1, CL1)] = CR1_CL1;
    // addEdge(M, CR2);    routeMapping[make_pair(M, CR2)] = M_CR2;        routeMapping[make_pair(CR2, M)] = CR2_M;
    ADD_EDGE(CR1, CR3);
    // //Triple skips
    // addEdge(CL3, M);    routeMapping[make_pair(CL3, M)] = CL3_M;        routeMapping[make_pair(M, CL3)] = M_CL3;
    // addEdge(CL2, CR1);  routeMapping[make_pair(CL2, CR1)] = CL2_CR1;    routeMapping[make_pair(CR1, CL2)] = CR1_CL2;
    // addEdge(CL1, CR2);  routeMapping[make_pair(CL1, CR2)] = CL1_CR2;    routeMapping[make_pair(CR2, CL1)] = CR2_CL1;
    // addEdge(M, CR3);    routeMapping[make_pair(M, CR3)] = M_CR3;        routeMapping[make_pair(CR3, M)] = CR3_M;
    // //Quadruple skips
    // addEdge(CL3, CR1);  routeMapping[make_pair(CL3, CR1)] = CL3_CR1;    routeMapping[make_pair(CR1, CL3)] = CR1_CL3;
    // addEdge(CL2, CR2);  routeMapping[make_pair(CL2, CR2)] = CL2_CR2;    routeMapping[make_pair(CR2, CL2)] = CR2_CL2;
    // addEdge(CL1, CR3);  routeMapping[make_pair(CL1, CR3)] = CL1_CR3;    routeMapping[make_pair(CR3, CL1)] = CR3_CL1;
    // //Quintuple skips   
    // addEdge(CL3, CR2);  routeMapping[make_pair(CL3, CR2)] = CL3_CR2;    routeMapping[make_pair(CR2, CL3)] = CR2_CL3;
    // addEdge(CL2, CR3);  routeMapping[make_pair(CL2, CR3)] = CL2_CR3;    routeMapping[make_pair(CR3, CL2)] = CR3_CL2;
    // //Sixtuple skips
    // addEdge(CL3, CR3);  routeMapping[make_pair(CL3, CR3)] = CL3_CR3;    routeMapping[make_pair(CR3, CL3)] = CR3_CL3;
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
        switchLifoRobotPosition(30, currentAlignment, endAlignment);
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
        robot.arc(45, -92, 0, COAST);
    }
    else if(turnDifference == 1 || turnDifference == -3)
    {
        DEBUGPRINT("Turning right 90 (Central)\n");
        robot.arc(45, 92, 0, COAST);
    }
    else if(turnDifference == -2 || turnDifference == 2)
    {
        DEBUGPRINT("Reversing (Central)\n");
        robot.arc(45, 183, 0, COAST);
    }
}

//length = 1 -> room <-> midpoint || length = 2 -> room <-> room
void rooms_vertical(orientation target, int length, bool isTargetRed)
{
    lineDetectionMode detector = isTargetRed ? COLORED : NORMAL;
    standardTurn(currentDirection, target, CENTERED);

    if(length == 1)
    {
        lifoRoute1Line(CENTERED, 9, 3, 5, 0, 30, detector);
    }
    else if(length == 2)
    {
        lifoRoute1Line(CENTERED, 28, 3, 5, 0, 40, detector);
    }
    else
    {
        DEBUGPRINT("ERROR, route length specified for vertical room tranfer incompatible!!");
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

    setLifo("SL", "SR");

    lifoUnregExtreme.distance(30, 10, NONE);
    lifoUnregNormal.lines(40, 1, NONE, 10);

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
    robot.arc(45, 50, 15, NONE);
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

//From and to M
orientation M_CL1(orientation dir)
{
    DEBUGPRINT("\nM_CL1\n");

    centralTurn(currentDirection, WEST);
    robot.setLinearAccelParams(100, 0, 30);
    robot.straight(45, 15, NONE);
    switchLifoRobotPosition(20, currentAlignment, CENTERED);
    lifoRoute1Line(CENTERED, 18, 7, 5, 0, 40, NORMAL);

    return WEST;
}
orientation CL1_M(orientation dir)
{
    DEBUGPRINT("\nCL1_M\n");

    standardTurn(currentDirection, EAST, CENTERED);
    lifoRoute1Line(CENTERED, 10, 5, 0, 3, 30, NORMAL);
    robot.setMode(CONTROLLED);
    robot.setLinearAccelParams(100, 30, 0);
    robot.straight(45, 21, COAST);
    
    return EAST;
}
orientation M_CR1(orientation dir)
{
    DEBUGPRINT("\nM_CR1\n");

    centralTurn(currentDirection, EAST);
    robot.setLinearAccelParams(100, 0, 30);
    robot.arc(30, 15, 20, NONE);
    robot.setLinearAccelParams(100, 30, 30);
    robot.straight(45, 12, NONE);
    currentAlignment = RIGHT_OF_LINE;
    lifoRoute1Line(RIGHT_OF_LINE, 18, 7, 5, 0, 40, NORMAL);

    return EAST;
}
orientation CR1_M(orientation dir)
{
    DEBUGPRINT("\nCR1_M\n");

    standardTurn(currentDirection, WEST, LEFT_OF_LINE);
    lifoRoute1Line(LEFT_OF_LINE, 10, 5, 0, 3, 30, NORMAL);
    robot.setMode(CONTROLLED);
    robot.setLinearAccelParams(100, 20, 0);
    robot.straight(45, 20, COAST);

    return WEST;
}
orientation M_TM(orientation dir)
{
    DEBUGPRINT("\nM_TM\n");

    centralTurn(currentDirection, NORTH);
    robot.setLinearAccelParams(100, 0, 30);
    robot.straight(45, 17, NONE);
    lifoRoute1Line(CENTERED, 13, 7, 5, 0, 30, NORMAL);

    return NORTH;
}
orientation TM_M(orientation dir)
{
    DEBUGPRINT("\nTM_M\n");

    standardTurn(currentDirection, SOUTH, CENTERED);
    lifoRoute1Line(CENTERED, 7, 5, 0, 2, 30, NORMAL);
    robot.setMode(CONTROLLED);
    robot.setLinearAccelParams(100, 30, 0);
    robot.straight(45, 21, COAST);

    return SOUTH;
}
orientation M_BM(orientation dir)
{
    DEBUGPRINT("\nM_BM\n");

    centralTurn(currentDirection, SOUTH);
    robot.setLinearAccelParams(100, 0, 30);
    robot.straight(45, 17, NONE);
    lifoRoute1Line(CENTERED, 13, 7, 5, 0, 30, NORMAL);

    return SOUTH;
}
orientation BM_M(orientation dir)
{
    DEBUGPRINT("\nBM_M\n");

    standardTurn(currentDirection, NORTH, CENTERED);
    lifoRoute1Line(CENTERED, 7, 5, 0, 2, 30, NORMAL);
    robot.setMode(CONTROLLED);
    robot.setLinearAccelParams(100, 30, 0);
    robot.straight(45, 21, COAST);

    return NORTH;
}

//From and to CL1
orientation CL1_TL(orientation dir)
{
    DEBUGPRINT("\nCL1_TL\n");

    standardTurn(currentDirection, NORTH, RIGHT_OF_LINE);
    lifoRoute1Line(RIGHT_OF_LINE, 15, 3, 5, 0, 40, SCANNER, NONE);

    display.resetScreen();
    display.format("%  \n")%static_cast<int>(scannedValue);

    robot.setLinearAccelParams(100, 30, 30);
    robot.arc(45, 45, -3, COAST);
    robot.arc(45, 20, 25, COAST);
    setLifo("70", "SR");
    stopScanning = false;
    act_tsk(HUMAN_SCAN_TASK);
    tslp_tsk(1);
    lifoUnregExtreme.distance(30, 13, COAST);
    stopScanning = true;

    display.format("%  \n")%static_cast<int>(scannedValue);

    lifoUnregNormal.seconds(30, 0.5, COAST);

    return NORTH;
}
orientation TL_CL1(orientation dir)
{
    DEBUGPRINT("\nTL_CL1\n");

    standardTurn(currentDirection, SOUTH, CENTERED);
    lifoRoute1Line(CENTERED, 23, 5, 5, 0, 40, NORMAL);

    return SOUTH;
}
orientation CL1_BL(orientation dir)
{
    DEBUGPRINT("\nCL1_BL\n");

    standardTurn(currentDirection, SOUTH, CENTERED);
    lifoRoute1Line(CENTERED, 23, 3, 5, 0, 40, NORMAL);

    return SOUTH;
}
orientation BL_CL1(orientation dir)
{
    DEBUGPRINT("\nBL_CL1\n");

    standardTurn(currentDirection, NORTH, CENTERED);
    lifoRoute1Line(CENTERED, 23, 3, 5, 0, 40, NORMAL);

    return NORTH;
}
orientation CL1_CL2(orientation dir)
{
    DEBUGPRINT("\nCL1_CL2\n");

    standardTurn(currentDirection, WEST, CENTERED);
    lifoRoute1Line(CENTERED, 17, 3, 5, 0, 40, NORMAL);

    return WEST;
}
orientation CL2_CL1(orientation dir)
{
    DEBUGPRINT("\nCL2_CL1\n");

    standardTurn(currentDirection, EAST, CENTERED);
    lifoRoute1Line(CENTERED, 17, 3, 5, 0, 40, NORMAL);

    return EAST;
}

//From and to CL2
orientation CL2_CL3(orientation dir)
{
    DEBUGPRINT("\nCL2_CL3\n");

    standardTurn(currentDirection, WEST, CENTERED);
    lifoRoute1Line(CENTERED, 8, 3, 5, 0, 30, NORMAL);

    return WEST;
}
orientation CL3_CL2(orientation dir)
{
    DEBUGPRINT("\nCL3_CL2\n");

    standardTurn(currentDirection, EAST, CENTERED);
    lifoRoute1Line(CENTERED, 8, 3, 5, 0, 30, NORMAL);

    return EAST;
}
orientation CL2_YR1(orientation dir)
{
    DEBUGPRINT("\nCL2_YR1\n");

    rooms_vertical(NORTH, 1);

    return NORTH;
}
orientation YR1_CL2(orientation dir)
{
    DEBUGPRINT("\nYR1_CL2\n");

    rooms_vertical(SOUTH, 1);

    return SOUTH;
}
orientation CL2_BR1(orientation dir)
{
    DEBUGPRINT("\nCL2_BR1\n");

    rooms_vertical(SOUTH, 1);

    return SOUTH;
}
orientation BR1_CL2(orientation dir)
{
    DEBUGPRINT("\nBR1_CL2\n");

    rooms_vertical(NORTH, 1);

    return NORTH;
}

//From and to CL3
orientation CL3_YR2(orientation dir)
{
    DEBUGPRINT("\nCL3_YR2\n");

    rooms_vertical(NORTH, 1);

    return NORTH;
}
orientation YR2_CL3(orientation dir)
{
    DEBUGPRINT("\nYR2_CL3\n");

    rooms_vertical(SOUTH, 1);

    return SOUTH;
}
orientation CL3_BR2(orientation dir)
{
    DEBUGPRINT("\nCL3_BR2\n");

    rooms_vertical(SOUTH, 1);

    return SOUTH;
}
orientation BR2_CL3(orientation dir)
{
    DEBUGPRINT("\nBR2_CL3\n");

    rooms_vertical(NORTH, 1);

    return NORTH;
}

//From and to CR1
orientation CR1_TR(orientation dir)
{
    DEBUGPRINT("\nCR1_TR\n");

    standardTurn(currentDirection, NORTH, CENTERED);
    lifoRoute1Line(CENTERED, 23, 3, 5, 0, 40, NORMAL);

    return NORTH;
}
orientation TR_CR1(orientation dir)
{
    DEBUGPRINT("\nTR_CR1\n");

    standardTurn(currentDirection, SOUTH, CENTERED);
    lifoRoute1Line(CENTERED, 23, 3, 5, 0, 40, NORMAL);

    return SOUTH;
}
orientation CR1_BR(orientation dir)
{
    DEBUGPRINT("\nCR1_BR\n");

    standardTurn(currentDirection, SOUTH, LEFT_OF_LINE);
    lifoRoute1Line(LEFT_OF_LINE, 35, 5, 5, 0, 40, NO_DETECT);

    return SOUTH;
}
orientation BR_CR1(orientation dir)
{
    DEBUGPRINT("\nBR_CR1\n");

    standardTurn(currentDirection, NORTH, RIGHT_OF_LINE);
    lifoRoute1Line(RIGHT_OF_LINE, 23, 5, 5, 0, 40, NORMAL);

    return NORTH;
}
orientation CR1_CR2(orientation dir)
{
    DEBUGPRINT("\nCR1_CR2\n");

    standardTurn(currentDirection, EAST, CENTERED);
    lifoRoute1Line(CENTERED, 17, 3, 5, 0, 40, NORMAL);

    return EAST;
}
orientation CR2_CR1(orientation dir)
{
    DEBUGPRINT("\nCR2_CR1\n");

    standardTurn(currentDirection, WEST, CENTERED);
    lifoRoute1Line(CENTERED, 17, 3, 5, 0, 40, NORMAL);

    return WEST;
}

//From and to CR2
orientation CR2_CR3(orientation dir)
{
    DEBUGPRINT("\nCR2_CR3\n");

    standardTurn(currentDirection, EAST, CENTERED);
    lifoRoute1Line(CENTERED, 8, 3, 5, 0, 30, NORMAL);

    return EAST;
}
orientation CR3_CR2(orientation dir)
{
    DEBUGPRINT("\nCR3_CR2\n");

    standardTurn(currentDirection, WEST, CENTERED);
    lifoRoute1Line(CENTERED, 8, 3, 5, 0, 30, NORMAL);

    return WEST;
}
orientation CR2_GR1(orientation dir)
{
    DEBUGPRINT("\nCR2_GR1\n");

    rooms_vertical(SOUTH, 1);

    return SOUTH;
}
orientation GR1_CR2(orientation dir)
{
    DEBUGPRINT("\nGR1_CR2\n");

    rooms_vertical(NORTH, 1);

    return NORTH;
}
orientation CR2_RR1(orientation dir)
{
    DEBUGPRINT("\nCR2_RR1\n");

    rooms_vertical(NORTH, 1, true);

    return NORTH;
}
orientation RR1_CR2(orientation dir)
{
    DEBUGPRINT("\nRR1_CR2\n");

    rooms_vertical(SOUTH, 1);

    return SOUTH;
}

//From and to CR3
orientation CR3_GR2(orientation dir)
{
    DEBUGPRINT("\nCR3_GR2\n");

    rooms_vertical(SOUTH, 1);

    return SOUTH;
}
orientation GR2_CR3(orientation dir)
{
    DEBUGPRINT("\nGR2_CR3\n");

    rooms_vertical(NORTH, 1);

    return NORTH;
}
orientation CR3_RR2(orientation dir)
{
    DEBUGPRINT("\nCR3_RR2\n");

    rooms_vertical(NORTH, 1, true);

    return NORTH;
}
orientation RR2_CR3(orientation dir)
{
    DEBUGPRINT("\nRR2_CR3\n");

    rooms_vertical(SOUTH, 1);

    return SOUTH;
}

//Human Routes
orientation M_TLH(orientation dir)
{
    DEBUGPRINT("\nM_TLH\n");

    centralTurn(dir, NORTH);
    robot.setMode(CONTROLLED);
    robot.setLinearAccelParams(100, 0, 0);
    robot.arc(45, -23, 0, COAST);
    robot.straight(45, 27, COAST);

    return NO;
}
orientation TLH_M(orientation dir)
{
    DEBUGPRINT("\nTLH_M\n");

    robot.setMode(CONTROLLED);
    robot.setLinearAccelParams(100, 0, 0);
    robot.straight(45, -27, COAST);
    robot.arc(45, 23, 0, COAST);

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

    rooms_vertical(NORTH, 2);

    return NORTH;
}
orientation YR1_BR1(orientation dir)
{
    DEBUGPRINT("\nYR1_BR1\n");

    rooms_vertical(SOUTH, 2);

    return SOUTH;
}
orientation BR2_YR2(orientation dir)
{
    DEBUGPRINT("\nBR2_YR2\n");

    rooms_vertical(NORTH, 2);

    return NORTH;
}
orientation YR2_BR2(orientation dir)
{
    DEBUGPRINT("\nYR2_BR2\n");

    rooms_vertical(SOUTH, 2);

    return SOUTH;
}

orientation GR1_RR1(orientation dir)
{
    DEBUGPRINT("\nGR1_RR1\n");

    rooms_vertical(NORTH, 2, true);

    return NORTH;
}
orientation RR1_GR1(orientation dir)
{
    DEBUGPRINT("\nRR1_GR1\n");

    rooms_vertical(SOUTH, 2);

    return SOUTH;
}
orientation GR2_RR2(orientation dir)
{
    DEBUGPRINT("\nGR2_RR2\n");

    rooms_vertical(NORTH, 2, true);

    return NORTH;
}
orientation RR2_GR2(orientation dir)
{
    DEBUGPRINT("\nRR2_GR2\n");

    rooms_vertical(SOUTH, 2);

    return SOUTH;
}

//Double skips
orientation CL3_CL1(orientation dir)
{
    DEBUGPRINT("\nCL3_CL1\n");

    standardTurn(currentDirection, EAST, CENTERED);
    lifoRoute1Line(CENTERED, 38, 3, 5, 0, 40, NORMAL);

    return EAST;
}
orientation CL1_CL3(orientation dir)
{
    DEBUGPRINT("\nCL1_CL3\n");

    standardTurn(currentDirection, WEST, CENTERED);
    lifoRoute1Line(CENTERED, 38, 3, 5, 0, 40, NORMAL);

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

    standardTurn(currentDirection, EAST, CENTERED);
    lifoRoute1Line(CENTERED, 38, 3, 5, 0, 40, NORMAL);

    return EAST;
}
orientation CR3_CR1(orientation dir)
{
    DEBUGPRINT("\nCR3_CR1\n");

    standardTurn(currentDirection, WEST, CENTERED);
    lifoRoute1Line(CENTERED, 38, 3, 5, 0, 40, NORMAL);

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
