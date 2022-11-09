#include "extras.h"
#include "globalRobot.h"

using namespace std;
using namespace ev3cxx;
using namespace ev3ys;

human::human(matPos pos)
{
    color = NO_COLOR;
    position = pos;
    isColorSet = false;
    isPicked = false;
    
    currentState = UNVISITED;

    switch (pos)
    {
    case TLLH:
        sprintf(name, "TLLH");
        break;
    case TLH:
        sprintf(name, "TLH");
        break;
    case TRRH:
        sprintf(name, "TRRH");
        break;
    case TRH:
        sprintf(name, "TRH");
        break;
    case BLLH:
        sprintf(name, "BLLH");
        break;
    case BLH:
        sprintf(name, "BLH");
        break;
    case BRRH:
        sprintf(name, "BRRH");
        break;
    case BRH:
        sprintf(name, "BRH");
        break;
    default:
        sprintf(name, "invalid");
        break;
    }   
}

void human::report()
{
    DEBUGPRINT("\nHuman originally located in %s ", name);
    //TODO: make it more analytical
    DEBUGPRINT("is in state %d and of color %d\n", currentState, color);
}

void human::grabHuman()
{
    grabber.moveDegrees(600, 150, BRAKE);
    isPicked = true; 
}

void initializeHumans()
{
    humans.insert(pair<matPos, human>(TLLH, human(TLLH)));
    humans.insert(pair<matPos, human>(TLH, human(TLH)));
    humans.insert(pair<matPos, human>(BLLH, human(BLLH)));
    humans.insert(pair<matPos, human>(BLH, human(BLH)));
    humans.insert(pair<matPos, human>(TRRH, human(TRRH)));
    humans.insert(pair<matPos, human>(TRH, human(TRH)));
    humans.insert(pair<matPos, human>(BRRH, human(BRRH)));
    humans.insert(pair<matPos, human>(BRH, human(BRH)));
}

void inferLastHuman()
{
    int numOfCols = 8;
    int appearances[7];
    
    for(int i = 0; i < 7; i++) appearances[i] = 0;
    
    for(auto x : humans)
    {
        if(x.second.getColorSet())
        {
            appearances[static_cast<int>(x.second.getColor())]++;
        }
    }
    for(auto &x : humans)
    {
        if(!x.second.getColorSet())
        {
            for(int i = 0; i < 7; i++)
            {
                if(appearances[i] == 0) 
                    x.second.setColor(static_cast<colors>(i));
            }
        }
    }
}

void loadHuman()
{
    act_tsk(PICK_BLOCK_TASK);
    tslp_tsk(1);
}
void releaseHuman()
{
    act_tsk(OPEN_GRABBER_TASK);
    tslp_tsk(1);
    robot.setMode(CONTROLLED);
    robot.setLinearAccelParams(100, 0, 0);
    robot.straight(40, 10, COAST);
    robot.setLinearAccelParams(200, 0, -40);
    robot.straight(40, -5, NONE);
    robot.setLinearAccelParams(200, -40, 0);
    act_tsk(PICK_BLOCK_TASK);
    tslp_tsk(1);
    robot.straight(40, -5, NONE);
}
