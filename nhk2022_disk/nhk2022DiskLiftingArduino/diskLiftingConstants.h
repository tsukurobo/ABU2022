#pragma once
#include <Arduino.h>

constexpr uint8_t MOVABLE_MD_NO = 0x17;
constexpr uint8_t ARM_MD_NO = 0x15;

constexpr int MOVABLE_TOUCH_PIN = 3;
constexpr int ARM_TOUCH_PIN = 5;

constexpr int DEFAULT_PWM = 140;

constexpr int MOVABLE_LOWEST = 70;                                          //[mm]
constexpr int MOVABLE_LENGTH = 620;                                         //[mm]
constexpr int ARM_GAP = 162;                                                //[mm]
constexpr long MOVABLE_MOTION_RANGE = 505;                                  //[mm]
constexpr long ARM_MOTION_RANGE = 450;                                      //[mm]
constexpr int BASIC_ARM_HEIGHT = MOVABLE_LOWEST + MOVABLE_LENGTH - ARM_GAP; //[mm]
// constexpr int HEIGHTS[] = {100, 300, 500, 700, 900, 1100};                  //[mm]
constexpr int HEIGHTS[] = {200, 400, 600, 800, 1000, 1100}; //[mm]

constexpr auto MOVABLE_BELT_RADIUS = 11; //[mm]
constexpr auto ARM_BELT_RADIUS = 9.56;      //[mm]

// constexpr auto RADIUS = 11.0; //[mm]
constexpr auto PPR = 2048 * 2;
// constexpr auto ARM_PPR = 2048; // Pulse Per Revolution
// constexpr auto MOVABLE_PPR=1024;//Pulse Per Revolution
// constexpr double CONVERTING_COEFFICIENT = 2 * RADIUS * PI / PPR;

// constexpr long ACCETABLE_ERROR = 360; // Pulse
constexpr long ACCETABLE_ERROR = 20; //[mm]