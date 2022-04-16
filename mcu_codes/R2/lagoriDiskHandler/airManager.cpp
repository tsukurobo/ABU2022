#pragma once
#include "airManager.h"


Air::Air(int pinL, int pinR) : pinL(pinL), pinR(pinR)
{
}
void Air::setState(const int &newState)
{
    state = newState;
    switch (state)
    {
    case 0:
        digitalWrite(pinL, LOW);
        digitalWrite(pinR, LOW);
        break;
    case 1:
        digitalWrite(pinL, HIGH);
        digitalWrite(pinR, LOW);
        break;
    case 2:
        digitalWrite(pinL, LOW);
        digitalWrite(pinR, HIGH);
        break;

    default:
        break;
    }
    stateModifiedTime = millis();
}

void Air::checkTimePassed() { checkTimePassed(DEFAULT_AIR_TIME); }
void Air::checkTimePassed(const unsigned long time)
{
    if (getState() != 0 && millis() - stateModifiedTime >= time)
    {
        setState(0);
    }
}