//joyconからの指令を変換してstmに送るノード

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <abu2022_msgs/BaseCmd.h>
#include "omni/key_mapping_elecom.h"
// #include <abu2022_msgs/BaseData.h>
#include <math.h>

class OmniCommander
{
private:
    enum RoleFlag
    {
        SEEKER = 0,
        HITTER
    }role_flag_;

    ros::Publisher vel_pub_;
    ros::Subscriber joy_sub_;
    // ros::Subscriber omni_sub_;
    abu2022_msgs::BaseCmd ard_cmd_;

    double max_lin_vel_ = 0, max_rot_vel_ = 0;  //最大並進速度と回転速度。単位はm/sとrad/s
    double max_lin_vel_slow_ = 0, max_rot_vel_slow_ = 0,  //ゆっくりめの並進速度と回転速度。単位はm/sとrad/s
           max_lin_vel_fast_ = 0, max_rot_vel_fast_ = 0;  //速めの並進速度と回転速度
    double zero_thresh_ = 0.1;
    // double omni_theta_ = 0; //単位はrad

public:
    void joyCb(const sensor_msgs::Joy::ConstPtr &msg)
    {
        //do not switch mode (press windows key) while using sticks
        if(msg->WINDOWS == 1 /* && msg->EIGHT == 1 */)
        {
            role_flag_ = (RoleFlag)(!role_flag_);
            max_lin_vel_ = (role_flag_ == HITTER) ? max_lin_vel_fast_ : max_lin_vel_slow_;
            max_rot_vel_ = (role_flag_ == HITTER) ? max_rot_vel_fast_ : max_rot_vel_slow_;

            ROS_INFO("omni_commander: switched to %s mode", (role_flag_ == HITTER) ? "hitter" : "seeker");
            return;
        }

        double vx = max_lin_vel_*msg->axes[1], vy = max_lin_vel_*msg->axes[0],
            v_mag = sqrt(vx*vx + vy*vy), omega = max_rot_vel_*msg->axes[3];
        
        //速度ベクトルの大きさを制限
        if(v_mag > max_lin_vel_)
        {
            vx = vx*max_lin_vel_/v_mag;
            vy = vy*max_lin_vel_/v_mag;
        }

        //zero_thresh未満の値は0として扱う
        if(fabs(vx) < zero_thresh_) vx = 0;
        if(fabs(vy) < zero_thresh_) vy = 0;
        if(fabs(omega) < zero_thresh_) omega = 0;
        //オドメトリ座標系における速度ベクトルで操作できるようにする
        // ard_cmd_.vx = vx*cos(omni_theta_) + vy*sin(omni_theta_);
        // ard_cmd_.vy = -vx*sin(omni_theta_) + vy*cos(omni_theta_);
        ard_cmd_.vx = vx;
        ard_cmd_.vy = vy;
        ard_cmd_.omega = omega;

        //2度送るのは、1度だけだとマイコンまで届かないことが稀にあるため
        vel_pub_.publish(ard_cmd_);
        vel_pub_.publish(ard_cmd_);
    }

    // void omniCb(const abu2022_msgs::BaseData::ConstPtr &data)
    // {
    //     omni_theta_ = data->theta;
    // }

    void startSpin()
    {
        ros::spin();
    }

    OmniCommander(ros::NodeHandle &nh)
    : role_flag_(HITTER)
    {
        vel_pub_ = nh.advertise<abu2022_msgs::BaseCmd>("cmd", 100);
        joy_sub_ = nh.subscribe("joy", 100, &OmniCommander::joyCb, this);
        // omni_sub_ = nh.subscribe("omni_info", 100, &OmniCommander::omniCb, this);

        while(!nh.getParam("omni/max_linear_vel_fast", max_lin_vel_fast_));
        while(!nh.getParam("omni/max_rotation_vel_fast", max_rot_vel_fast_));
        while(!nh.getParam("omni/max_linear_vel_slow", max_lin_vel_slow_));
        while(!nh.getParam("omni/max_rotation_vel_slow", max_rot_vel_slow_)); 

        //default values of max_lin_vel_ and max_rot_vel_ are set to that of "fast" version
        max_lin_vel_ = max_lin_vel_fast_;
        max_rot_vel_ = max_rot_vel_fast_;
    }

    /*~OmniCommander()
    {
    }*/
};



int main(int argc, char **argv)
{
    ros::init(argc, argv, "omni_commander");
    ros::NodeHandle nh;
    OmniCommander oc(nh);

    oc.startSpin();    
}
