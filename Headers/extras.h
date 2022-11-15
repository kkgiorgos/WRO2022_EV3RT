#pragma once

#include "routes.h"
#include <map>
#include "ev3ys.h"

class human
{
public:
    enum state
    {
        UNVISITED,
        SCANNED,
        PICKED,
        LOADED,
        DONE
    };

private:
    ev3ys::colors color;
    matPos position;
    char name[8];
    state currentState;
    bool isColorSet;
    bool isPicked;

public:
    human() {}
    human(matPos pos);

    void report();

    void setColor(ev3ys::colors color);
    ev3ys::colors getColor() {return color;}

    void setPos(matPos pos) {position = pos;}
    matPos getPos() {return position;}

    bool getColorSet() {return isColorSet;}

    void grabHuman();

    bool getIsPicked() {return isPicked;}
};

extern std::map<matPos, human> humans;

void initializeHumans();
void inferLastHuman();

void loadHuman();
void releaseHuman();
