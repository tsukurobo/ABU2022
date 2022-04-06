//joyconからの指令を変換してstmに送るノード

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <abu2022_msgs/BaseCmd.h>
// #include <abu2022_msgs/BaseData.h>
#include <math.h>

class OmniCommander
{
private:
    ros::Publisher vel_pub_;
    ros::Subscriber joy_sub_;
    // ros::Subscriber omni_sub_;
    abu2022_msgs::BaseCmd ard_cmd_;

    double max_lin_vel_ = 0, max_rot_vel_ = 0;  //最大並進速度と回転速度。ともに単位はm/s
    // double omni_theta_ = 0; //単位はrad

public:
    void joyCb(const sensor_msgs::Joy::ConstPtr &msg)
    {
        double vx = max_lin_vel_*msg->axes[1], vy = max_lin_vel_*msg->axes[0],
            v_mag = sqrt(vx*vx + vy*vy);
        
        //速度ベクトルの大きさを制限
        if(v_mag > max_lin_vel_)
        {
            vx = vx*max_lin_vel_/v_mag;
            vy = vy*max_lin_vel_/v_mag;
        }

        //オドメトリ座標系における速度ベクトルで操作できるようにする
        // ard_cmd_.vx = vx*cos(omni_theta_) + vy*sin(omni_theta_);
        // ard_cmd_.vy = -vx*sin(omni_theta_) + vy*cos(omni_theta_);
        ard_cmd_.vx = vx;
        ard_cmd_.vy = vy;
        ard_cmd_.omega = max_rot_vel_*msg->axes[3];

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
    {
        vel_pub_ = nh.advertise<abu2022_msgs::BaseCmd>("cmd", 100);
        joy_sub_ = nh.subscribe("joy", 100, &OmniCommander::joyCb, this);
        // omni_sub_ = nh.subscribe("omni_info", 100, &OmniCommander::omniCb, this);

        while(!nh.getParam("omni/max_linear_vel", max_lin_vel_));
        while(!nh.getParam("omni/max_rotation_vel", max_rot_vel_));
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
