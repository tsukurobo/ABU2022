//R1射出・装填・照準機構制御用ROS側プログラム

#include <ros/ros.h>
#include "shooter/key_mapping_elecom.h"
#include <abu2022_msgs/R1ArduinoCmd.h>
#include <sensor_msgs/Joy.h>

//#define PUSHED 1

class Shooter
{
private:
    enum ButtonState {PUSHED=1};
    enum LeverState
    {
        OPEN=0,
        CLOSE
    };
    enum MotorState
    {
        ROTATE = 0,
        STOP
    };
    enum ShooterActuator
    {
        PointerMotor = 0,
        RollerUpDownMotor,
        RollerMotor1,
        RollerMotor2,
        LeverServo1,
        LeverServo2,
        LeverServo3
    };
    ros::Publisher pub_;
    ros::Subscriber sub_;
    abu2022_msgs::R1ArduinoCmd ard_cmd_;
    
    //パラメータ
    int roller_rot_speed_ = 0;
    int roller_updown_speed_ = 0;
    int lever_angles_[3][2] = {0};
    int pointer_rot_speed_ = 0;

    //1, 3, 4キーを押すたびに動作が変わるようにするための変数(0 or 1)
    LeverState lever_angle_selector_[3] = {LeverState::OPEN};
    MotorState roller_motor_state_ = MotorState::STOP;

public:
    //ジョイコンからのデータが来たときに呼ばれる関数
    void joyCb(const sensor_msgs::Joy &joymsg)
    //joyCb = [this](const sensor_msgs::Joy &joymsg) -> void
    {
        //十字キー左右で照準機構を回転させる
        ard_cmd_.actuator_id = ShooterActuator::PointerMotor;
        ard_cmd_.value = pointer_rot_speed_*joymsg.AXES_LR;
        pub_.publish(ard_cmd_);

        //十字キー上下でローラー機構の上下方向の向きを変化させる
        ard_cmd_.actuator_id = ShooterActuator::RollerUpDownMotor;
        ard_cmd_.value = roller_updown_speed_*joymsg.AXES_UD;
        pub_.publish(ard_cmd_);

        //1キーで装填レバー1を開閉
        if(joymsg.ONE == ButtonState::PUSHED)
        {
            ard_cmd_.actuator_id = ShooterActuator::LeverServo1;
            ard_cmd_.value = lever_angles_[0][lever_angle_selector_[0]];
            pub_.publish(ard_cmd_);

            lever_angle_selector_[0] = static_cast<LeverState>(1 - lever_angle_selector_[0]);
        }

        //4キーで装填レバー2を開閉
        if(joymsg.FOUR == ButtonState::PUSHED)
        {
            ard_cmd_.actuator_id = ShooterActuator::LeverServo2;
            ard_cmd_.value = lever_angles_[1][lever_angle_selector_[1]];
            pub_.publish(ard_cmd_);

            lever_angle_selector_[1] = static_cast<LeverState>(1 - lever_angle_selector_[1]);
        }

        //3キーで装填レバー3を開閉
        if(joymsg.THREE == ButtonState::PUSHED)
        {   
            ard_cmd_.actuator_id = ShooterActuator::LeverServo3;
            ard_cmd_.value = lever_angles_[2][lever_angle_selector_[2]];
            pub_.publish(ard_cmd_);

            lever_angle_selector_[2] = static_cast<LeverState>(1 - lever_angle_selector_[2]);
        }

        //2キーでローラーの回転・停止
        if(joymsg.TWO == ButtonState::PUSHED)
        {
            ard_cmd_.actuator_id = ShooterActuator::RollerMotor1;
            ard_cmd_.value = (roller_motor_state_ == MotorState::STOP) ? roller_rot_speed_ : 0;
            pub_.publish(ard_cmd_);

            ard_cmd_.actuator_id = ShooterActuator::RollerMotor2;
            ard_cmd_.value = (roller_motor_state_ == MotorState::STOP) ? -roller_rot_speed_ : 0;
            pub_.publish(ard_cmd_);

            roller_motor_state_ = static_cast<MotorState>(1 - roller_motor_state_);
        }
    }

    Shooter(ros::NodeHandle &nh)
    {
        //パブリッシャ・サブスクライバの初期化
        pub_ = nh.advertise<abu2022_msgs::R1ArduinoCmd>("arduino_cmd", 50);
        sub_ = nh.subscribe("joy", 100, &Shooter::joyCb, this);
        
        //パラメータの設定
        while(!nh.getParam("pointer/rotation_speed", pointer_rot_speed_)){ROS_INFO("error0");}
        while(!nh.getParam("roller/rotation_speed", roller_rot_speed_)){ROS_INFO("error1");}
        while(!nh.getParam("roller/up_down_speed", roller_updown_speed_)){ROS_INFO("error2");}
        while(!nh.getParam("loader/lever1_angle_open", lever_angles_[0][0])){ROS_INFO("error3");}
        while(!nh.getParam("loader/lever1_angle_close", lever_angles_[0][1])){ROS_INFO("error4");}
        while(!nh.getParam("loader/lever2_angle_open", lever_angles_[1][0])){ROS_INFO("error5");}
        while(!nh.getParam("loader/lever2_angle_close", lever_angles_[1][1])){ROS_INFO("error6");}
        while(!nh.getParam("loader/lever3_angle_open", lever_angles_[2][0])){ROS_INFO("error7");}
        while(!nh.getParam("loader/lever3_angle_close", lever_angles_[2][1])){ROS_INFO("error8");}
    }

    void startSpin()
    {
        ros::spin();
    }

};

int main(int argc, char **argv)
{
    //ROSノードの初期化
    ros::init(argc, argv, "shooter");
    ros::NodeHandle nh;

    Shooter shooter(nh);

    shooter.startSpin();

}