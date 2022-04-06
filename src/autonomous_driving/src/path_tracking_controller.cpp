#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Joy.h>
#include <fstream>
#include "autonomous_driving/key_mapping_elecom.h"

class PathTrackingController
{
private:
    ros::Publisher path_pub_;
    ros::Publisher en_tracking_pub_;

    std_msgs::Bool flag_msg_;
    nav_msgs::Path path_loaded_;

public:
    PathTrackingController(ros::NodeHandle &nh)
    {
        path_pub_ = nh.advertise<nav_msgs::Path>("path", 1);
        en_tracking_pub_ = nh.advertise<std_msgs::Bool>("enable_tracking", 1);
    }

    //load csv file and set values of path_loaded_
    void publishPath(const std::string &path_filename)
    {
        geometry_msgs::PoseStamped p;
        std::ifstream file(path_filename);

        // check if the file has been opened correctly
        if(file.fail()){
            ROS_FATAL("Failed to open file\n");
            return;
        }

        //reset path_loaded_
        path_loaded_.poses.clear();

        while(1){
            std::string waypoint_x, waypoint_y, speed_cmd, yaw_cmd;

            //get data from the loaded csv file
            getline(file, waypoint_x, ',');
            getline(file, waypoint_y, ',');
            getline(file, speed_cmd, ',');
            getline(file, yaw_cmd);

            //check if current position has reached the end of the file
            if(file.eof()) break;
            
            //std::cout << waypoint_x << " " << waypoint_y << " " << speed_cmd << " " << yaw_cmd << std::endl;
            //add new nwaypoint information to path_loaded_
            p.pose.position.x = std::stod(waypoint_x);
            p.pose.position.y = std::stod(waypoint_y);
            p.pose.orientation.x = std::stod(speed_cmd);
            p.pose.orientation.y = std::stod(yaw_cmd);
            path_loaded_.poses.push_back(p);
        }

        path_loaded_.header.frame_id = "map";

        path_pub_.publish(path_loaded_);

        file.close();

        ROS_INFO("path_tracking_controller: published a path %s", path_filename.c_str());
    }   

    void publishTrackingStartFlag(bool flag)
    {
        flag_msg_.data = flag;
        en_tracking_pub_.publish(flag_msg_);

        ROS_INFO("path_tracking_controller: send a command %d", flag);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_loader");
    ros::NodeHandle nh;
    
    PathTrackingController ptc(nh);
    std::string path_file1, path_file2;

    boost::function<void (const sensor_msgs::Joy::ConstPtr &)> joyCb = 
    [&](const sensor_msgs::Joy::ConstPtr &msg) -> void
    {
        if(msg->ONE == 1)
        {
            ptc.publishPath(path_file1);
        }
        else if(msg->TWO == 1)
        {
            ptc.publishPath(path_file2);
        }
        else if(msg->THREE == 1)
        {
            ptc.publishTrackingStartFlag(true);
        }
        else if(msg->FOUR == 1)
        {
            ptc.publishTrackingStartFlag(false);
        }
    };

    ros::Subscriber joy_sub = nh.subscribe("joy", 10, joyCb);

    while(!nh.getParam("pure_pursuit/path1", path_file1)){ROS_ERROR("could not get the param");}
    while(!nh.getParam("pure_pursuit/path2", path_file2)){ROS_ERROR("could not get the param");}

    ros::spin();

    return 0;
}