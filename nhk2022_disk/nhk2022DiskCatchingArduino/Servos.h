#pragma once
#include <Servo.h>

class Servos
{
private:
    Servo servoL;
    Servo servoR;

public:
    void attach(const int &leftPin, const int &rightPin)
    {
        servoL.attach(leftPin);
        servoR.attach(rightPin);
    }
    void write(const int &pwm)
    {
        servoL.write(pwm);
        servoR.write(pwm);
    }
    int read() { return servoL.read(); }
    int read(const bool &left) { return left ? servoL.read() : servoR.read(); }
    void writeMicroseconds(const int &value)
    {
        servoL.writeMicroseconds(value);
        servoR.writeMicroseconds(value);
    }
};