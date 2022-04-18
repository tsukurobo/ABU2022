#pragma once
#include <ros.h>
#include <std_msgs/Int16MultiArray.h>
#include "ise_motor_driver_v3.h"
#include "airManager.h"
#include "diskLiftingBelt.h"

ros::NodeHandle nh;
namespace
{
    int prevCommandType = 2;
    int prevCommandA = 1;
    int prevCommandB = 1;
}

IseMotorDriver movableMd(MOVABLE_MD_NO);
IseMotorDriver armMd(ARM_MD_NO);

Movable movable(1, MOVABLE_TOUCH_PIN, MOVABLE_MOTION_RANGE, -1, MOVABLE_UPWARD_PWM, MOVABLE_DOWNWARD_PWM, MOVABLE_BELT_RADIUS);
Arm arm(-1, ARM_TOUCH_PIN, ARM_MOTION_RANGE, 1, ARM_UPWARD_PWM, ARM_DOWNWARD_PWM, ARM_BELT_RADIUS, movable);
Air diskKeeper(KEEPER_AIR_PIN1, KEEPER_AIR_PIN2);
Air armRotater(ROTATION_AIR_PIN1, ROTATION_AIR_PIN2);

// std_msgs::Int32MultiArray pub_msg;
// ros::Publisher pub("diskStatus", &pub_msg);

/**
 * @brief モーターの駆動を直ちに停止
 *
 * @param stopMovable 可動レールを停止するか
 * @param stopArm アームを停止するか
 */
void stop(const int &stopMovable, const int &stopArm);

void messageCb(const std_msgs::Int16MultiArray &command_msg)
{
    String info = "Received a message of type " + String(command_msg.data[0]);
    nh.logwarn(info.c_str());
    switch (command_msg.data[0])
    {
    case 0:
        if (command_msg.data[1] == true && (prevCommandType != 0 || prevCommandA == false))
            movable.startInitialization();
        if (command_msg.data[2] == true && (prevCommandType != 0 || prevCommandB == false))
            arm.startInitialization();
        break;
        /*     case 1:
                if ((prevCommandType != 1 || prevCommandA != command_msg.data[1]) && command_msg.data[1] > -1)
                {
                    arm.setTargetHeight(HEIGHTS[command_msg.data[1]]);
                    arm.setMode(1);
                    movable.setMode(1);
                }
                break; */
    case 2:
        if (prevCommandType != 2 || prevCommandA != command_msg.data[1] || prevCommandB != command_msg.data[2])
            stop(command_msg.data[1], command_msg.data[2]);
        break;
    case 3:
        movable.setManualDirection(command_msg.data[1]);
        arm.setManualDirection(command_msg.data[2]);
        break;
    case 4:
        if (command_msg.data[1] != 0)
        {
//            String info = "" + String(command_msg.data[1]);
//            nh.logwarn(info.c_str());
            diskKeeper.setState(command_msg.data[1]);
        }
        else if (command_msg.data[1] == 0 || diskKeeper.getState() != 0)
        {
            diskKeeper.setState(0);
        }
        break;
    case 5:
        if (command_msg.data[1] != 0)
        {
            armRotater.setState(command_msg.data[1]);
        }
        else if (command_msg.data[1] == 0 || armRotater.getState() != 0)
        {
            armRotater.setState(0);
        }
        break;
    default:
        break;
    }
    prevCommandType = command_msg.data[0];
    prevCommandA = command_msg.data[1];
    prevCommandB = command_msg.data[2];
}
ros::Subscriber<std_msgs::Int16MultiArray> sub("liftDisk", &messageCb);

void setup()
{
    nh.initNode();
    nh.subscribe(sub);

    IseMotorDriver::begin();
    if (!movableMd == false)
        movableMd << IseMotorDriver::createSettingData(PWM_64KHZ, SM_BRAKE_LOW_SIDE);
    if (!armMd == false)
        armMd << IseMotorDriver::createSettingData(PWM_64KHZ, SM_BRAKE_LOW_SIDE);

    pinMode(MOVABLE_TOUCH_PIN, INPUT_PULLUP);
    pinMode(ARM_TOUCH_PIN, INPUT_PULLUP);

    // pub_msg.data = (int32_t *)malloc(sizeof(int32_t) * 5);
    // pub_msg.data_length = 5;

    Serial.begin(115200);
}

void loop()
{
    nh.spinOnce();
    /*     if (bool isInAutoMoving = movable.getMode() == 1 || arm.getMode() == 1)
        {
            long currentArmHeight = arm.getCurrentHeightInMM();
            long targetHeight = arm.getTargetHeight();
            bool ifDirectionUpward = currentArmHeight < targetHeight;
            int autoDirection = 0;
            if (ifDirectionUpward)
            {
                autoDirection = (currentArmHeight < targetHeight) ? 1 : 0;
            }
            else
            {
                autoDirection = (currentArmHeight > targetHeight + ACCEPTABLE_ERROR) ? -1 : 0;
            }

            if (arm.getMode() == 1)
            {
                arm.setAutoDirection(autoDirection);
                if (autoDirection == 0)
                {
                    arm.immediateStop();
                }
            }
            if (movable.getMode() == 1)
            {
                movable.setAutoDirection(autoDirection);
                if (autoDirection == 0)
                {
                    movable.immediateStop();
                }
            }
        } */

    if (movable.ifTouch())
    {
        movable.setCurrentPositionAsInitialPosition();
        if (movable.ifImmediateStopPossible())
        {
            movable.immediateStop();
        }
        //nh.loginfo("Movable is in Touch");
    }
    if (arm.ifTouch())
    {
        arm.setCurrentPositionAsInitialPosition();
        if (arm.ifImmediateStopPossible())
        {
            arm.immediateStop();
        }
        //nh.loginfo("Arm is in Touch");
    }

    movableMd << movable.pwm();
    armMd << arm.pwm();
    movableMd >> movable.encoder;
    armMd >> arm.encoder;

    diskKeeper.checkTimePassed();
    armRotater.checkTimePassed();

    // Logging
    /*nh.logwarn("LOOP");
    String mode = "Working Mode. Arm: " + String(arm.getMode()) + " Movable: " + String(movable.getMode());
    nh.loginfo(mode.c_str());
    String armPWM = "Arm PWM: " + String(arm.pwm());
    nh.loginfo(armPWM.c_str());
    String movablePWM = "Movable PWM: " + String(movable.pwm());
    nh.loginfo(movablePWM.c_str());
    String armPulse = "Arm Pulse: " + String(arm.encoder);
    nh.loginfo(armPulse.c_str());
    String movablePulse = "Movable Pulse: " + String(movable.encoder);
    nh.loginfo(movablePulse.c_str());
    String armTarget = "Target Height: " + String(arm.getTargetHeight());
    nh.loginfo(armTarget.c_str());
    String armHeight = "Arm Height: " + String(arm.getCurrentHeightInMM());
    nh.loginfo(armHeight.c_str());
    String movableHeight = "Movable Height: " + String(movable.getCurrentHeightInMM());
    nh.loginfo(movableHeight.c_str());
    String armMovement = "Arm Movement: " + String(arm.getCurrentMovementInMM());
    nh.loginfo(armMovement.c_str());
    String movableMovement = "Movable Movement: " + String(movable.getCurrentMovementInMM());
    nh.loginfo(movableMovement.c_str());*/

    // pub_msg.data[0] = arm.getCurrentHeightInMM();
    // pub_msg.data[1] = movable.getCurrentMovementInMM();
    // pub_msg.data[2] = arm.getCurrentMovementInMM();
    // pub_msg.data[3] = movable.getMode();
    // pub_msg.data[4] = arm.getMode();
    // pub.publish(&pub_msg);
    delay(10);
}

void stop(const int &stopMovable, const int &stopArm)
{
    if (stopMovable)
    {
        movable.immediateStop();
    }
    if (stopArm)
    {
        arm.immediateStop();
    }
}
