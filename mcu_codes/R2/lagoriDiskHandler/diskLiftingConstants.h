#pragma once
#include <Arduino.h>

constexpr uint8_t MOVABLE_MD_NO = 0x17;
constexpr uint8_t ARM_MD_NO = 0x15;

constexpr int MOVABLE_TOUCH_PIN = 3;
constexpr int ARM_TOUCH_PIN = 5;

//// constexpr int DEFAULT_PWM = 140;
//constexpr int MOVABLE_UPWARD_PWM = -200;
//constexpr int MOVABLE_DOWNWARD_PWM = 140;
//constexpr int ARM_UPWARD_PWM = 140;
//constexpr int ARM_DOWNWARD_PWM = -140;

constexpr int MOVABLE_UPWARD_PWM = -200;
constexpr int MOVABLE_DOWNWARD_PWM = 250;
constexpr int ARM_UPWARD_PWM = 170;
constexpr int ARM_DOWNWARD_PWM = -200;

constexpr int MOVABLE_LOWEST = 70;                                //[mm]
constexpr int MOVABLE_LENGTH = 620;                               //[mm]
constexpr int ARM_GAP = 100;                                      //[mm]
constexpr long MOVABLE_MOTION_RANGE = 400;                        //[mm]
constexpr long ARM_MOTION_RANGE = 450;                            //[mm]
//constexpr long ARM_MOTION_RANGE = 600;                            //[mm]
constexpr long ARM_ADDITIONAL_RANGE_WHEN_CATCHER_HORIZONTAL = 150; //[mm]
constexpr int HEIGHTS[] = {200, 400, 600, 800, 1000, 1100};       //[mm]

constexpr auto MOVABLE_BELT_RADIUS = 14.67; //[mm]
constexpr auto ARM_BELT_RADIUS = 9.56;   //[mm]

constexpr auto PPR = 2048 * 2;
constexpr long ACCEPTABLE_ERROR = 20; //[mm]
