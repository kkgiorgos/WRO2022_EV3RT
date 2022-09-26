#pragma once

#define V 36

#include <vector>
#include <queue>
#include <cstdio>
#include <climits>
#include "ev3ys.h"
#include "methods.h"

//Graph - Routing Code
enum matPos
{
    //Main network
    S = 0,
    W = 1, L = 2,
    G = 3, R = 4, B = 5, Y = 6,

    //Surprise network
    M = 7, BM = 8, TM = 9, BL = 10, BR = 11, TL = 12, TR = 13,
    CL1 = 14, CL2 = 15, CL3 = 16,
    CR1 = 17, CR2 = 18, CR3 = 19,
    GR1 = 20, GR2 = 21, RR1 = 22, RR2 = 23, BR1 = 24, BR2 = 25, YR1 = 26, YR2 = 27,
    TLLH = 28, TLH = 29, TRH = 30, TRRH = 31, BLLH = 32, BLH = 33, BRH = 34, BRRH = 35
};

enum orientation
{
    NORTH,
    EAST,
    SOUTH,
    WEST,
    NO
};

extern matPos currentPos;
extern orientation currentDirection;
extern lifoRobotPosition currentAlignment;

typedef std::pair<int, int> weightedVertex;
typedef orientation (*routeFunc)( orientation );

extern std::vector<int> graph[V];

void addEdge(int u, int v);
void graphInit();
int dijkstra(int source, int target, std::vector<int> *path);

void constructRoute(routeFunc *route, std::vector<int> *pathNodes, int distance);

orientation executeRoute(routeFunc *route, int distance, orientation dir);

void fullRoute(int source, int target, orientation dir);
void fullRouteStandard(int target);

//Turn functions
void standardTurn(orientation start, orientation finish, lifoRobotPosition endAlignment);

//Actual route implementations
//Main network
orientation S_W(orientation dir);
orientation W_G(orientation dir);
orientation G_R(orientation dir);
orientation R_B(orientation dir);
orientation B_Y(orientation dir);
orientation Y_L(orientation dir);
orientation L_S(orientation dir);

//Full network standard for surprise use
orientation M_CL1(orientation dir);
orientation M_CR1(orientation dir);
orientation M_TM(orientation dir);
orientation M_BM(orientation dir);

orientation CL1_M(orientation dir);
orientation CL1_CL2(orientation dir);
orientation CL1_TL(orientation dir);
orientation CL1_BL(orientation dir);

orientation CR1_M(orientation dir);
orientation CR1_CR2(orientation dir);
orientation CR1_TR(orientation dir);
orientation CR1_BR(orientation dir);

orientation TM_M(orientation dir);

orientation BM_M(orientation dir);

orientation TL_CL1(orientation dir);

orientation TR_CR1(orientation dir);

orientation BL_CL1(orientation dir);

orientation BR_CR1(orientation dir);

orientation CL2_CL1(orientation dir);
orientation CL2_CL3(orientation dir);
orientation CL2_YR1(orientation dir);
orientation CL2_BR1(orientation dir);

orientation CR2_CR1(orientation dir);
orientation CR2_CR3(orientation dir);
orientation CR2_GR1(orientation dir);
orientation CR2_RR1(orientation dir);

orientation CL3_CL2(orientation dir);
orientation CL3_YR2(orientation dir);
orientation CL3_BR2(orientation dir);

orientation CR3_CR2(orientation dir);
orientation CR3_GR2(orientation dir);
orientation CR3_RR2(orientation dir);

orientation YR1_CL2(orientation dir);
orientation YR2_CL3(orientation dir);

orientation BR1_CL2(orientation dir);
orientation BR2_CL3(orientation dir);

orientation GR1_CR2(orientation dir);
orientation GR2_CR3(orientation dir);

orientation RR1_CR2(orientation dir);
orientation RR2_CR3(orientation dir);

orientation M_TLH(orientation dir);
orientation TLH_M(orientation dir);
orientation M_TRH(orientation dir);
orientation TRH_M(orientation dir);
orientation M_BLH(orientation dir);
orientation BLH_M(orientation dir);
orientation M_BRH(orientation dir);
orientation BRH_M(orientation dir);
orientation TL_TLLH(orientation dir);
orientation TLLH_TL(orientation dir);
orientation TR_TRRH(orientation dir);
orientation TRRH_TR(orientation dir);
orientation BL_BLLH(orientation dir);
orientation BLLH_BL(orientation dir);
orientation BR_BRRH(orientation dir);
orientation BRRH_BR(orientation dir);
