#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Int16MultiArray.h"

ros::Publisher diskLiftingPub;
//ros::Publisher diskCatchingPub;
ros::Subscriber sub;

std_msgs::Int16MultiArray pubLiftingCommand;
std_msgs::Int16MultiArray prePubLiftingCommand;

// std_msgs::Int16MultiArray pubDiskCatchingCommand;
// std_msgs::Int16MultiArray prePubDiskCatchingCommand;

float prevCrossTB = 0;
float prevCrossLR = 0;
bool prevStart = 0;
bool prevBack = 0;

bool ifPrevAutomation = 0;
bool ifPrevManualRotation = 0;
bool ifPrevDiskAir = 0;

namespace JCU4113S
{
    constexpr int CROSS_TB = 5;
    constexpr int CROSS_LR = 4;
    constexpr int ONE = 0;
    constexpr int TWO = 1;
    constexpr int THREE = 2;
    constexpr int FOUR = 3;
    constexpr int LB = 4;
    constexpr int RB = 5;
    constexpr int LT = 6;
    constexpr int RT = 7;
    constexpr int BACK = 10;
    constexpr int START = 11;
}

void joyCallback(const sensor_msgs::Joy &joyMsg)
{
    using namespace JCU4113S;
    //ROS_INFO("Call back");

    //移動命令
    if (joyMsg.buttons[LB] == false && joyMsg.buttons[LT] == false)
    {
        ROS_INFO("L buttons are not pressed.");
        // if (joyMsg.buttons[ONE])
        // {
        //     // 100[mm]
        //     pubLiftingCommand.data[0] = 1;
        //     pubLiftingCommand.data[1] = (joyMsg.buttons[RB] ? 4 : 0);
        //     ROS_INFO("Auto movement");
        // }
        // else if (joyMsg.buttons[TWO])
        // {
        //     // 300[mm]
        //     pubLiftingCommand.data[0] = 1;
        //     pubLiftingCommand.data[1] = (joyMsg.buttons[RB] ? 5 : 1);
        //     ROS_INFO("Auto movement");
        // }
        // else if (joyMsg.buttons[THREE])
        // {
        //     // 500[mm]
        //     pubLiftingCommand.data[0] = 1;
        //     pubLiftingCommand.data[1] = 2;
        //     ROS_INFO("Auto movement");
        // }
        // else if (joyMsg.buttons[FOUR])
        // {
        //     // 700[mm]
        //     pubLiftingCommand.data[0] = 1;
        //     pubLiftingCommand.data[1] = 3;
        //     ROS_INFO("Auto movement");
        // }
        // else if (/* ifPrevAutomation */ prePubLiftingCommand.data[0]==1&&prePubLiftingCommand.data[1]!=-1)
        // {
        //     pubLiftingCommand.data[0] = 1;
        //     pubLiftingCommand.data[1] = -1;
        //     ROS_INFO("Released auto movement");
        // }
        // ifPrevAutomation = (pubLiftingCommand.data[0] == 1 && !ifPrevAutomation);

        //2, 4キー -> 把持用エアシリンダ作動
        // if (joyMsg.axes[CROSS_LR] > 0.0)
        if (joyMsg.buttons[TWO])
        {
            // pubDiskCatchingCommand.data[0] = 0;
            // pubDiskCatchingCommand.data[1] = 1;
            pubLiftingCommand.data[0]=4;
            pubLiftingCommand.data[1]=1;
        }
        // else if (joyMsg.axes[CROSS_LR] < 0.0)
        else if (joyMsg.buttons[FOUR])
        {
            // pubDiskCatchingCommand.data[0] = 0;
            // pubDiskCatchingCommand.data[1] = 2;
            pubLiftingCommand.data[0]=4;
            pubLiftingCommand.data[1]=2;
        }

        //手動調整命令
        if (joyMsg.axes[CROSS_TB] > 0.0)
        {
            pubLiftingCommand.data[0] = 3;
            pubLiftingCommand.data[1] = (joyMsg.buttons[RT] ? 0 : 1);
            pubLiftingCommand.data[2] = (joyMsg.buttons[RB] ? 0 : 1);
            ROS_INFO("Manual movement");
        }
        else if (joyMsg.axes[CROSS_TB] < 0.0)
        {
            pubLiftingCommand.data[0] = 3;
            pubLiftingCommand.data[1] = (joyMsg.buttons[RT] ? 0 : -1);
            pubLiftingCommand.data[2] = (joyMsg.buttons[RB] ? 0 : -1);
            ROS_INFO("Manual movement");
        }
        else if (prevCrossTB != 0.0)
        {
            pubLiftingCommand.data[0] = 3;
            pubLiftingCommand.data[1] = (0);
            pubLiftingCommand.data[2] = (0);
            ROS_INFO("Manual movement");
        }
        prevCrossTB = joyMsg.axes[CROSS_TB];

        //イニシャライズ命令
        if (joyMsg.buttons[START])
        {
            pubLiftingCommand.data[0] = 0;
            pubLiftingCommand.data[1] = (joyMsg.buttons[RT] ? 0 : 1);
            pubLiftingCommand.data[2] = (joyMsg.buttons[RB] ? 0 : 1);
            ROS_INFO("Initialization");
        }
        else
        {
            if (prevStart)
            {
                pubLiftingCommand.data[0] = 0;
                pubLiftingCommand.data[1] = 0;
                pubLiftingCommand.data[2] = 0;
            }
        }
        prevStart = joyMsg.buttons[START];

        //停止命令
        if (joyMsg.buttons[BACK])
        {
            pubLiftingCommand.data[0] = 2;
            pubLiftingCommand.data[1] = (joyMsg.buttons[RT] ? 0 : 1);
            pubLiftingCommand.data[2] = (joyMsg.buttons[RB] ? 0 : 1);
            ROS_INFO("Stop");
        }
        else if (prevBack)
        {
            pubLiftingCommand.data[0] = 2;
            pubLiftingCommand.data[1] = 0;
            pubLiftingCommand.data[2] = 0;
        }
        prevBack = joyMsg.buttons[BACK];
    }
    else
    {
        //LB + 2, 4キー -> 回転用エアシリンダ作動
        if (joyMsg.buttons[LB])
        {
            if (joyMsg.buttons[TWO])
            {
                // pubDiskCatchingCommand.data[0] = 1;
                // pubDiskCatchingCommand.data[1] = 2;
                pubLiftingCommand.data[0]=5;
                pubLiftingCommand.data[1]=1;
            }
            else if (joyMsg.buttons[FOUR])
            {
                // pubDiskCatchingCommand.data[0] = 1;
                // pubDiskCatchingCommand.data[1] = 1;
                pubLiftingCommand.data[0]=5;
                pubLiftingCommand.data[1]=2;
            }
        }
        // if (joyMsg.buttons[LT])
        // {
        //     if (joyMsg.buttons[ONE])
        //     {
        //         // pubDiskCatchingCommand.data[0] = 2;
        //         // pubDiskCatchingCommand.data[1] = 1;
        //         ifPrevManualRotation = true;
        //     }
        //     else if (joyMsg.buttons[THREE])
        //     {
        //         // pubDiskCatchingCommand.data[0] = 2;
        //         // pubDiskCatchingCommand.data[1] = -1;
        //         ifPrevManualRotation = true;
        //     }
        //     else if (ifPrevManualRotation)
        //     {
        //         // pubDiskCatchingCommand.data[0] = 2;
        //         // pubDiskCatchingCommand.data[1] = 0;
        //         ifPrevManualRotation = false;
        //     }
        // }

        // // if (joyMsg.axes[CROSS_LR] > 0.0)
        // if (joyMsg.buttons[TWO])
        // {
        //     // pubDiskCatchingCommand.data[0] = 0;
        //     // pubDiskCatchingCommand.data[1] = 1;
        //     pubLiftingCommand.data[0]=4;
        //     pubLiftingCommand.data[1]=1;
            
        // }
        // // else if (joyMsg.axes[CROSS_LR] < 0.0)
        // else if (joyMsg.buttons[FOUR])
        // {
        //     // pubDiskCatchingCommand.data[0] = 0;
        //     // pubDiskCatchingCommand.data[1] = 2;
        //     pubLiftingCommand.data[0]=4;
        //     pubLiftingCommand.data[1]=2;
        // }
        // else if (ifPrevDiskAir)
        // {
        //     // pubDiskCatchingCommand.data[0] = 0;
        //     // pubDiskCatchingCommand.data[1] = 0;
        //     pubLiftingCommand.data[0]=4;
        //     pubLiftingCommand.data[1]=0;
        // }
        // ifPrevDiskAir = (pubDiskCatchingCommand.data[0] == 0 && !ifPrevDiskAir);
        // prevCrossLR = joyMsg.axes[CROSS_LR];
    }
}

int main(int argc, char **argv)
{
    pubLiftingCommand.data.resize(3);
    prePubLiftingCommand.data.resize(3);

    pubLiftingCommand.data[0] = 2;
    pubLiftingCommand.data[1] = 1;
    pubLiftingCommand.data[2] = 1;
    prePubLiftingCommand.data[0] = 2;
    prePubLiftingCommand.data[1] = 1;
    prePubLiftingCommand.data[2] = 1;

    //pubDiskCatchingCommand.data.resize(2);
    //prePubDiskCatchingCommand.data.resize(2);

    ros::init(argc, argv, "joy_sub_node");
    ros::NodeHandle nh;
    diskLiftingPub = nh.advertise<std_msgs::Int16MultiArray>("liftDisk", 10);
    //diskCatchingPub = nh.advertise<std_msgs::Int16MultiArray>("catchDisk", 10);
    sub = nh.subscribe("joy", 10, joyCallback);

    ros::Rate loop_rate(50);
    while (ros::ok())
    {
        ros::spinOnce();

        if (pubLiftingCommand.data[0] != prePubLiftingCommand.data[0] || pubLiftingCommand.data[1] != prePubLiftingCommand.data[1] || pubLiftingCommand.data[2] != prePubLiftingCommand.data[2] /* || (ifPrevAutomation && pubLiftingCommand.data[0] == 1 && pubLiftingCommand.data[1] == -1) */)
        {
            diskLiftingPub.publish(pubLiftingCommand);
            diskLiftingPub.publish(pubLiftingCommand);
            prePubLiftingCommand = pubLiftingCommand;
        }
        // if (pubDiskCatchingCommand.data[0] != prePubDiskCatchingCommand.data[0] || pubDiskCatchingCommand.data[1] != prePubDiskCatchingCommand.data[1])
        // {
        //     diskCatchingPub.publish(pubDiskCatchingCommand);
        //     diskCatchingPub.publish(pubDiskCatchingCommand);
        //     prePubDiskCatchingCommand = pubDiskCatchingCommand;
        // }
        loop_rate.sleep();
    }
    return 0;
}
