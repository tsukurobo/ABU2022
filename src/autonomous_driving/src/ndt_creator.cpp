#include <ros/ros.h>
//#include <nav_msgs/OccupancyGrid.h>
#include "autonomous_driving/ndt_original.h"

int main(int argc, char **argv)
{
    boost::shared_ptr<nav_msgs::OccupancyGrid const> map_ptr;

    //ROSの初期化
    ros::init(argc, argv, "ndt_creator");
    ros::NodeHandle nh;
    
    //地図を取得
    ROS_INFO("ndt_creator: Waiting for a map...");
    try
    {
        map_ptr = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("map", nh);

        //NDTの作成
        if(map_ptr)
        {
            ROS_INFO("ndt_creator: Creating NDT and exporting it to a file...");
            if(NDT::createAndExportNDT(map_ptr, argv[1], atoi(argv[2])))
            {
                ROS_INFO("ndt_creator: Finished!");
            }
            else
            {
                ROS_ERROR("ndt_creator: Failed to create or export NDT. The file path you entered may be wrong.");
            }
        }
        else
        {
            ROS_ERROR("ndt_creator: Could not get a vaild map. Try again.");
        }
    }
    catch(custom_math::Matrix::MatrixCalcException ex)
    {
        ROS_ERROR("ndt_creator: %s", ex.what().c_str());
        //std::cout << ex.what() << std::endl;
    }
    catch(...)
    {
        ROS_ERROR("ndt_creator: An error has occured while processing a map.");
    }

    return 0;
}