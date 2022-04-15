#pragma once
#include "diskLiftingBelt.h"
#include "diskLiftingConstants.h"
#include <Arduino.h>
#include <ros.h>

extern ros::NodeHandle nh;

// Belt
long Belt::convertEncoderToMM(const long &encoder) const
{
    return static_cast<long>(static_cast<double>(encoder) * convertingCoefficient);
}
long Belt::convertMMToEncoder(const long &mm) const
{
    return static_cast<long>(static_cast<double>(mm) / convertingCoefficient);
}
long Belt::getCurrentMovementInMM() const { return convertEncoderToMM(getCurrentMovement()); }
bool Belt::ifTouch() const { return digitalRead(touchPin); }
bool Belt::ifImmediateStopPossible() const
{
    return (getMode() == 0 || (getMode() == 3 && manualDirection * pwmPositiveDirection == 1) || (getMode() == 1 && autoDirection * pwmPositiveDirection == 1));
}
int Belt::pwm()
{
    //イニシャライズ
    if (mode == 0)
    {
        return max(pwmUpward, pwmDownward);
    }
    //自動移動
    /*     else if (mode == 1)
        {
            if ((autoDirection * pwmPositiveDirection == 1) || (encoder < initialPosition + convertMMToEncoder(getMotionRange()) && autoDirection * pwmPositiveDirection == -1))
            {
                return static_cast<int>(DEFAULT_PWM * autoDirection * pwmPositiveDirection);
            }
            return 0;
        } */
    //手動制御
    else if (mode == 3)
    {
        if ((manualDirection * pwmPositiveDirection == 1) || (encoder < initialPosition + convertMMToEncoder(getMotionRange()) && manualDirection * pwmPositiveDirection == -1))
        {
            return static_cast<int>((manualDirection > 0) ? pwmUpward : pwmDownward);
        }

        if (encoder < initialPosition - 5 && manualDirection * pwmPositiveDirection == 1)
        {
            nh.logerror("It can no longer go back.");
        }
        else if (encoder > initialPosition + convertMMToEncoder(getMotionRange()) && manualDirection * pwmPositiveDirection == -1)
        {
            nh.logerror("It can no longer go forward.");
        }

        return 0;
    }
    //停止中
    else
    {
        return 0;
    }
}
Belt::Belt(int encoderPositiveDirection, int touchPin, long motionRange, int pwmPositiveDirection, int pwmUpward, int pwmDownward, double radius) : encoderPositiveDirection(encoderPositiveDirection), touchPin(touchPin), motionRange(motionRange), pwmPositiveDirection(pwmPositiveDirection), pwmUpward(pwmUpward), pwmDownward(pwmDownward), convertingCoefficient(2 * radius * PI / PPR) {}

// Movable
long Movable::getCurrentHeightInMM() const { return (MOVABLE_LOWEST + getCurrentMovementInMM()); }

// Arm
long Arm::getCurrentHeightInMM() const { return (movable.getCurrentHeightInMM() + MOVABLE_LENGTH - ARM_GAP - getCurrentMovementInMM()); }
Arm::Arm(int encoderPositiveDirection, int touchPin, long motionRange, int pwmPositiveDirection, int pwmUpward, int pwmDownward, double radius, Movable &movable) : Belt(encoderPositiveDirection, touchPin, motionRange, pwmPositiveDirection, pwmUpward, pwmDownward, radius), movable(movable) {}
