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
        }
        else if(firstNode == RR)
        {
            if(secondNode == FR) route[i] = RR_FR;
        }
        else if(firstNode == GR)
        {
            if(secondNode == FR) route[i] = GR_FR;
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

    return NORTH;
}

orientation W_CL(orientation dir)
{
    DEBUGPRINT("\nW_CL\n");

    return WEST;
}
orientation W_CR(orientation dir)
{
    DEBUGPRINT("\nW_CR\n");

    return EAST;
}

orientation L_S(orientation dir)
{
    DEBUGPRINT("\nL_S\n");
    
    return NO;
}

orientation CL_CR(orientation dir)
{
    DEBUGPRINT("\nCL_CR\n");

    return EAST;
}
orientation CL_FL(orientation dir)
{
    DEBUGPRINT("\nCL_FL\n");
    
    return WEST;
}
orientation CL_L(orientation dir)
{
    DEBUGPRINT("\nCL_L\n");
    
    return SOUTH;
}

orientation CR_CL(orientation dir)
{
    DEBUGPRINT("\nCR_CL\n");

    return WEST;
}
orientation CR_FR(orientation dir)
{
    DEBUGPRINT("\nCR_FR\n");
    
    return EAST;
}
orientation CR_L(orientation dir)
{
    DEBUGPRINT("\nCR_L\n");
    
    return SOUTH;
}

orientation FL_CL(orientation dir)
{
    DEBUGPRINT("\nFL_CL\n");

    return EAST;
}
orientation FL_YR(orientation dir)
{
    DEBUGPRINT("\nFL_YR\n");

    return NORTH;
}
orientation FL_BR(orientation dir)
{
    DEBUGPRINT("\nFL_BR\n");

    return SOUTH;
}

orientation FR_CR(orientation dir)
{
    DEBUGPRINT("\nFR_CR\n");

    return WEST;
}
orientation FR_RR(orientation dir)
{
    DEBUGPRINT("\nFR_RR\n");

    return NORTH;
}
orientation FR_GR(orientation dir)
{
    DEBUGPRINT("\nFR_GR\n");

    return SOUTH;
}

orientation YR_FL(orientation dir)
{
    DEBUGPRINT("\nYR_FL\n");

    return SOUTH;
}
orientation BR_FL(orientation dir)
{
    DEBUGPRINT("\nBR_FL\n");

    return NORTH;
}
orientation RR_FR(orientation dir)
{
    DEBUGPRINT("\nRR_FR\n");

    return SOUTH;
}
orientation GR_FR(orientation dir)
{
    DEBUGPRINT("\nGR_FR\n");

    return NORTH;
}
