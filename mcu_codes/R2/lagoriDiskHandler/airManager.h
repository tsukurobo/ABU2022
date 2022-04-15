#pragma once
#include "diskCatchingConstants.h"
#include <Arduino.h>

class Air
{
private:
    int pinL;
    int pinR;
    int state = 0;
    unsigned long stateModifiedTime;

public:
    Air(int pinL, int pinR);
    void setState(const int &newState);
    int getState() const;
    void checkTimePassed();
    void checkTimePassed(const unsigned long time);
};


inline int Air::getState() const
{
    return state;
}