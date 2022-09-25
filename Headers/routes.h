#pragma once

#define V 11

#include <vector>
#include <queue>
#include <cstdio>
#include <climits>
#include "ev3ys.h"

//Graph - Routing Code
enum matPos
{
	S = 0,
    W = 1, L = 2,
    YR = 3, RR = 4, GR = 5, BR = 6,
    FL = 7, CL = 8, CR = 9, FR = 10
};

enum orientation
{
    NORTH,
    EAST,
    SOUTH,
    WEST,
    NO
};

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
void standardTurn(orientation start, orientation finish);
void specialTurn(orientation start, orientation finish);

//Actual route implementations
orientation S_W(orientation dir);
orientation S_CL(orientation dir);
orientation S_CR(orientation dir);

orientation W_CL(orientation dir);
orientation W_CR(orientation dir);
orientation W_S(orientation dir);

orientation L_S(orientation dir);
orientation L_W(orientation dir);

orientation CL_CR(orientation dir);
orientation CL_FL(orientation dir);
orientation CL_L(orientation dir);

orientation CR_CL(orientation dir);
orientation CR_FR(orientation dir);
orientation CR_L(orientation dir);

orientation FL_CL(orientation dir);
orientation FL_YR(orientation dir);
orientation FL_BR(orientation dir);

orientation FR_CR(orientation dir);
orientation FR_RR(orientation dir);
orientation FR_GR(orientation dir);

orientation YR_FL(orientation dir);
orientation BR_FL(orientation dir);
orientation RR_FR(orientation dir);
orientation GR_FR(orientation dir);

orientation BR_YR(orientation dir);
orientation GR_RR(orientation dir);
