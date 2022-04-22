#pragma once
#include "diskCatchingConstants.h"
#include <Arduino.h>

class Air
{
private:
    int pinL;
    int pinR;
    int state = 0;
    bool isInitialForm = true;
    unsigned long stateModifiedTime;

public:
    Air(int pinL, int pinR);
    void setState(const int &newState);
    int getState() const;
    void checkTimePassed();
    void checkTimePassed(const unsigned long time);
    void setForm(bool);
};

inline int Air::getState() const
{
    return state;
}
inline void Air::setForm(bool newForm)
{
    isInitialForm = newForm;
}