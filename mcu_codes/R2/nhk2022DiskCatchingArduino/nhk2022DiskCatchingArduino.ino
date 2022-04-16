#include <ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <Servo.h>
#include "Servos.h"

namespace
{
    constexpr int AIR_PIN1 = 6;
    constexpr int AIR_PIN2 = 8;
    constexpr int8_t SERVO_L_PIN = 11;
    constexpr int8_t SERVO_R_PIN = 12;
    constexpr int targetServoAngles[] = {10, 90, 155};
    constexpr int servoMax = 155;
    constexpr int servoMin = 60;
}
namespace
{
    int airState = 0;
    int airCommand = 0;
    int type = 0;

    bool ifServosAreManual = false;
    int ServosManualState
    = 0;

    unsigned long airStateModifiedTime, currentTime;
}
std_msgs::Int16MultiArray command;

ros::NodeHandle nh;

Servos servos;

void messageCb(const std_msgs::Int16MultiArray &msg)
{
    if (msg.data[0] == 0)
    {
        if (msg.data[1] != 0)
        {
            airCommand = msg.data[1];
        }
        else if (airState != 0)
        {

            airCommand = 0;
        }
    }
    else if (msg.data[0] == 1)
    {
        servos.write(targetServoAngles[msg.data[1]]);
        ifServosAreManual = false;
    }
    else if (msg.data[0] == 2)
    {
        ifServosAreManual = true;
        ServosManualState = msg.data[1];
    }
}

ros::Subscriber<std_msgs::Int16MultiArray> sub("catchDisk", &messageCb);

void setup()
{
    Serial.begin(57600);
    pinMode(AIR_PIN1, OUTPUT);
    pinMode(AIR_PIN2, OUTPUT);
    digitalWrite(AIR_PIN1, LOW);
    digitalWrite(AIR_PIN2, LOW);
    servos.attach(SERVO_L_PIN, SERVO_R_PIN);
    airStateModifiedTime = currentTime = millis();
    nh.initNode();
    nh.subscribe(sub);
}

void loop()
{
    currentTime = millis();
    if (airCommand == 1)
    {
        digitalWrite(AIR_PIN1, HIGH);
        digitalWrite(AIR_PIN2, LOW);
        airState = 1;

        airStateModifiedTime = millis();
    }
    else if (airCommand == 2)
    {
        digitalWrite(AIR_PIN1, LOW);
        digitalWrite(AIR_PIN2, HIGH);
        airStateModifiedTime = millis();
        airState = 2;
    }
    else if (airState != 0)
    {
        currentTime = millis();
        if (currentTime - airStateModifiedTime >= 300)
        {
            digitalWrite(AIR_PIN1, LOW);
            digitalWrite(AIR_PIN2, LOW);
            airState = 0;
        }
    }
    if (ifServosAreManual)
    {
        if (servos.read() < servoMax && ServosManualState > 0)
        {
            servos.write(min(servoMax, servos.read() + 1));
        }
        else if (servos.read() > servoMin && ServosManualState < 0)
        {
            servos.write(max(servoMin, servos.read() - 1));
        }
        else if (ServosManualState == 0)
        {
            servos.write(servos.read());
        }
    }
    nh.spinOnce();
    Serial.println(servos.read());
    delay(10);
}
