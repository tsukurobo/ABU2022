#include "ros/ros.h"
#include "std_msgs/String.h"

/* メッセージの型名::ConstPointer  */
void chatterCallback(const std_msgs::String::ConstPtr &msg)
{
    ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "listener");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

    ros::spin();/* この中でCallbackが実行
    無限ループ
     */

    return 0;
}
