#pragma once

#define V 38

#define ADD_EDGE(X,Y) addEdge(X,Y);routeMapping[make_pair(X,Y)]=X##_##Y;routeMapping[make_pair(Y,X)]=Y##_##X;

#include <vector>
#include <queue>
#include <map>
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
    IR = 3, IL = 4,
    G = 5, R = 6, B = 7, Y = 8,

    //Surprise network
    M = 9, BM = 10, TM = 11, BL = 12, BR = 13, TL = 14, TR = 15,
    CL1 = 16, CL2 = 17, CL3 = 18,
    CR1 = 19, CR2 = 20, CR3 = 21,
    GR1 = 22, GR2 = 23, RR1 = 24, RR2 = 25, BR1 = 26, BR2 = 27, YR1 = 28, YR2 = 29,
    TLLH = 30, TLH = 31, TRH = 32, TRRH = 33, BLLH = 34, BLH = 35, BRH = 36, BRRH = 37
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
extern std::map<std::pair<matPos, matPos>, routeFunc> routeMapping;

void addEdge(int u, int v);
void graphInit();
int dijkstra(int source, int target, std::vector<int> *path);

void constructRoute(routeFunc *route, std::vector<int> *pathNodes, int distance);

orientation executeRoute(routeFunc *route, int distance, orientation dir);

void fullRoute(int source, int target, orientation dir);
void fullRouteStandard(int target);

//Turn functions
void standardTurn(orientation start, orientation finish, lifoRobotPosition endAlignment);
void centralTurn(orientation start, orientation finish);

//Same route simplification
void rooms_vertical(orientation start, orientation target, int length, bool isTargetRed = false); 


//Actual route implementations
//Main network
orientation S_W(orientation dir);
orientation W_IR(orientation dir);
orientation IR_G(orientation dir);
orientation G_R(orientation dir);
orientation R_IR(orientation dir);
orientation IR_IL(orientation dir);
orientation IL_B(orientation dir);
orientation B_Y(orientation dir);
orientation Y_IL(orientation dir);
orientation IL_L(orientation dir);
orientation L_S(orientation dir);

orientation CR3_R(orientation dir);
orientation R_CR3(orientation dir);
orientation CR3_G(orientation dir);
orientation G_CR3(orientation dir);
orientation CL3_B(orientation dir);
orientation B_CL3(orientation dir);
orientation CL3_Y(orientation dir);
orientation Y_CL3(orientation dir);

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

orientation TL_TLH(orientation dir);
orientation TLH_TL(orientation dir);
orientation M_TRH(orientation dir);
orientation TRH_M(orientation dir);
orientation TRH_CR1(orientation dir);
orientation CR1_TRH(orientation dir);
orientation BL_BLH(orientation dir);
orientation BLH_BL(orientation dir);
orientation BR_BRH(orientation dir);
orientation BRH_BR(orientation dir);
orientation TL_TLLH(orientation dir);
orientation TLLH_TL(orientation dir);
orientation TR_TRRH(orientation dir);
orientation TRRH_TR(orientation dir);
orientation BL_BLLH(orientation dir);
orientation BLLH_BL(orientation dir);
orientation BR_BRRH(orientation dir);
orientation BRRH_BR(orientation dir);

//Secondary routing (faster, but may require higher maintenance)
//Vertical
orientation BM_TM(orientation dir);
orientation TM_BM(orientation dir);

orientation BL_TL(orientation dir);
orientation TL_BL(orientation dir);

orientation BR_TR(orientation dir);
orientation TR_BR(orientation dir);

orientation BR1_YR1(orientation dir);
orientation YR1_BR1(orientation dir);
orientation BR2_YR2(orientation dir);
orientation YR2_BR2(orientation dir);

orientation GR1_RR1(orientation dir);
orientation RR1_GR1(orientation dir);
orientation GR2_RR2(orientation dir);
orientation RR2_GR2(orientation dir);

//Double skips
orientation CL3_CL1(orientation dir);
orientation CL1_CL3(orientation dir);
orientation CL2_M(orientation dir);
orientation M_CL2(orientation dir);
orientation CL1_CR1(orientation dir);
orientation CR1_CL1(orientation dir);
orientation M_CR2(orientation dir);
orientation CR2_M(orientation dir);
orientation CR1_CR3(orientation dir);
orientation CR3_CR1(orientation dir);

//Triple skips
orientation CL3_M(orientation dir);
orientation M_CL3(orientation dir);
orientation CL2_CR1(orientation dir);
orientation CR1_CL2(orientation dir);
orientation CL1_CR2(orientation dir);
orientation CR2_CL1(orientation dir);
orientation M_CR3(orientation dir);
orientation CR3_M(orientation dir);

//Quadruple skips
orientation CL3_CR1(orientation dir);
orientation CR1_CL3(orientation dir);
orientation CL2_CR2(orientation dir);
orientation CR2_CL2(orientation dir);
orientation CL1_CR3(orientation dir);
orientation CR3_CL1(orientation dir);

//Quintuple skips
orientation CL3_CR2(orientation dir);
orientation CR2_CL3(orientation dir);
orientation CL2_CR3(orientation dir);
orientation CR3_CL2(orientation dir);

//Sixtuple skips
orientation CL3_CR3(orientation dir);
orientation CR3_CL3(orientation dir);
