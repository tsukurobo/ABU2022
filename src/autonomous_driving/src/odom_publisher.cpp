#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <abu2022_msgs/BaseData.h>

class OdomPublisher
{
private:
    ros::Subscriber base_sub_;
    tf2_ros::TransformBroadcaster tb_;
    geometry_msgs::TransformStamped ts_;
    double lidar_pos_[3] = {0}, omni_pos_[3] = {0};
    double alpha_xy_ = 0, alpha_th_ = 0;    //これらはオドメトリの補正係数

    void baseCb(const abu2022_msgs::BaseData::ConstPtr &msg)
    {
        //そういえば座標変換処理をしていないので、実装する必要あり
        omni_pos_[0] += msg->delta_x*alpha_xy_;
        omni_pos_[1] += msg->delta_y*alpha_xy_;
        omni_pos_[2] += msg->delta_theta*alpha_th_;

        ts_.header.stamp = ros::Time::now();
        //ts_.header.frame_id = "odom";
        //ts_.child_frame_id = "base_link";
        ts_.transform.translation.x = omni_pos_[0];
        ts_.transform.translation.y = omni_pos_[1];
        //ts_.transform.translation.z = 0;
        //ts_.transform.rotation.x = 0;
        //ts_.transform.rotation.y = 0;
        ts_.transform.rotation.z = sin(omni_pos_[2]*0.5);
        ts_.transform.rotation.w = cos(omni_pos_[2]*0.5);
        tb_.sendTransform(ts_);        
    }

public:
    OdomPublisher(ros::NodeHandle &nh)
    {
        base_sub_ = nh.subscribe("omni_info", 100, &OdomPublisher::baseCb, this);

        while(!nh.getParam("scanmatch/lidar_pos_x", lidar_pos_[0]));
        while(!nh.getParam("scanmatch/lidar_pos_y", lidar_pos_[1]));
        while(!nh.getParam("scanmatch/lidar_pos_th", lidar_pos_[2]));
        while(!nh.getParam("odom/alpha_xy", alpha_xy_));
        while(!nh.getParam("odom/alpha_th", alpha_th_));
    }

    void start()
    {
        //base_linkからlaserのtfを出しておく
        ts_.header.stamp = ros::Time::now();
        ts_.header.frame_id = "base_link";
        ts_.child_frame_id = "laser";
        ts_.transform.translation.x = lidar_pos_[0];
        ts_.transform.translation.y = lidar_pos_[1];
        ts_.transform.translation.z = 0;
        ts_.transform.rotation.x = 0;
        ts_.transform.rotation.y = 0;
        ts_.transform.rotation.z = sin(lidar_pos_[2]*0.5);
        ts_.transform.rotation.w = cos(lidar_pos_[2]*0.5);
        tb_.sendTransform(ts_);

        //odomからbase_linkのtfの初期値を出しておく
        ts_.header.stamp = ros::Time::now();
        ts_.header.frame_id = "odom";
        ts_.child_frame_id = "base_link";
        ts_.transform.translation.x = 0;
        ts_.transform.translation.y = 0;
        ts_.transform.translation.z = 0;
        ts_.transform.rotation.x = 0;
        ts_.transform.rotation.y = 0;
        ts_.transform.rotation.z = 0;
        ts_.transform.rotation.w = 1.0;
        tb_.sendTransform(ts_);

        ros::spin();
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odom_pub");
    ros::NodeHandle nh;
    OdomPublisher odom_pub(nh);

    odom_pub.start();

    return 0;
}