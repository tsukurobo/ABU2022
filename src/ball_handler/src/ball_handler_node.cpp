//joyconからの指令を変換してarduinoに送るノード
#include <random>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int16MultiArray.h>
#include "ball_handler/key_mapping_elecom.h"

class BallHandler
{
private:
    enum Actuators
    {
        AIR_CYLINDER_EXPAND = 0,
        AIR_CYLINDER_LIFT,
        AIR_CYLINDER_CATCH,
        MOTOR_BOH,
        // MOTOR_BOH_POS,  //for position control in hitter mode

        ACTUATOR_SIZE
    };

    enum CmdArrayIndex
    {
        ACTUATOR_ID = 0,
        COMMAND
    };

    enum AirCylinderState
    {
        PUSH = 0,
        PULL,
        NUTRAL
    };

    enum ButtonState 
    {
        PUSHED = 1,
        RELEASED = 0
        };
    enum LRAxesState
    {
        LEFT = 1, 
        RIGHT = -1
    };

    enum RoleFlag
    {
        SEEKER = 0,
        HITTER
    }role_flag_;

    ros::Publisher cmd_pub_;
    ros::Subscriber joy_sub_;
    ros::Duration dura_;
    ros::Timer boh_timer_;
    // ros::Subscriber omni_sub_;
    std_msgs::Int16MultiArray cmd_msg_;
    std::mutex mtx_;
    std::mt19937 mt_;
    std::uniform_real_distribution<> urd_;

    //1コールバック前の状態を格納する変数
    // int axes_lr_pre_ = 0;
    // double axes_lr_pre_ = 0;
    bool boh_auto_rot_ = true,
         is_moving_ = false;
    ButtonState prev_key3_state_ = RELEASED,
                prev_listkey_state_ = RELEASED;

    //parameters
    int boh_rot_speed_ = 0;
    int boh_autorot_speed_ = 100;
    int boh_random_speed_stop_max_, 
        boh_random_speed_stop_min_,
        boh_random_speed_move_max_,
        boh_random_speed_move_min_;

    AirCylinderState states[Actuators::ACTUATOR_SIZE-2] = 
        {AirCylinderState::PULL};

    void publishCmdToArduino(Actuators id, int val)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        cmd_msg_.data[ACTUATOR_ID] = id;
        cmd_msg_.data[COMMAND] = val;
        //2度送るのは、1度だけだとマイコンまで届かないことが稀にあるため
        cmd_pub_.publish(cmd_msg_);
        cmd_pub_.publish(cmd_msg_);
    }

public:
    void joyCb(const sensor_msgs::Joy::ConstPtr &msg)
    {
        //3(A)キー押すたびにPUSH/PULL -> 一定時間待ってNUTRALとなるようにする
        
        //Hitterモードかつ3(A)キーが押されているとき
        if(role_flag_ == HITTER && msg->A == PUSHED && prev_key3_state_ == RELEASED)
        {
            prev_key3_state_ = PUSHED;
            //+5(LB)キーで展開用エアシリンダ作動 
            if(msg->LB == PUSHED)
            {
                publishCmdToArduino(AIR_CYLINDER_EXPAND, states[AIR_CYLINDER_EXPAND]);
                states[AIR_CYLINDER_EXPAND] = (AirCylinderState)(!states[AIR_CYLINDER_EXPAND]);

                dura_.sleep();

                publishCmdToArduino(AIR_CYLINDER_EXPAND, NUTRAL);
            }
            //+6(RB)キーで上下移動用エアシリンダ作動
            else if(msg->RB == PUSHED)
            {
                publishCmdToArduino(AIR_CYLINDER_LIFT, states[AIR_CYLINDER_LIFT]);
                states[AIR_CYLINDER_LIFT] = (AirCylinderState)(!states[AIR_CYLINDER_LIFT]);

                dura_.sleep();

                publishCmdToArduino(AIR_CYLINDER_LIFT, NUTRAL);
            }

            //他に何も押されていなければ、把持用エアシリンダを動かす
            else
            {
                publishCmdToArduino(AIR_CYLINDER_CATCH, states[AIR_CYLINDER_CATCH]);
                states[AIR_CYLINDER_CATCH] = (AirCylinderState)(!states[AIR_CYLINDER_CATCH]);

                dura_.sleep();

                publishCmdToArduino(AIR_CYLINDER_CATCH, NUTRAL);
            }
        }
        else if(role_flag_ == HITTER && msg->A == RELEASED && prev_key3_state_ == PUSHED)
            prev_key3_state_ = RELEASED;
        // //Seekerモードで、かつ十字キーの左右の状態が変化したとき
        // else if(role_flag_ == SEEKER && msg->AXES_LR != axes_lr_pre_)
        // {
        //     axes_lr_pre_ = msg->AXES_LR;
        //     publishCmdToArduino(MOTOR_BOH, msg->AXES_LR*boh_rot_speed_);
        //     //ROS_INFO("%d", (int)(msg->AXES_LR*boh_rot_speed_));
        // }
        //Seeker mode:
        //rotate BOH at the particular speed when the rotation command around yaw axes is not zero
        //rotate BOH at the random speed when the velocity command is zero
        else if(role_flag_ == SEEKER /*&& msg->STICK_R_LR != 0*/)
        {
            // if(msg->X == PUSHED)
            // {
            //     boh_auto_rot_ = !boh_auto_rot_;
            //     if(boh_auto_rot_ == false)
            //     {
            //         publishCmdToArduino(MOTOR_BOH, 0);
            //         boh_timer_.stop();
            //     }
            // }
            if(msg->STICK_R_LR != 0)
            {
                boh_timer_.stop();
                publishCmdToArduino(MOTOR_BOH, msg->STICK_R_LR*boh_autorot_speed_);
                // axes_lr_pre_ = msg->STICK_R_LR;
            }
            // else if(msg->STICK_R_LR == 0 && axes_lr_pre_ != 0)
            // {
            //     publishCmdToArduino(MOTOR_BOH, 0);
            //     axes_lr_pre_ = 0;
            // }
            else if(msg->STICK_L_LR != 0 || msg->STICK_L_UD != 0)
            {
                is_moving_ = true;

                if(boh_auto_rot_) boh_timer_.start();
                else publishCmdToArduino(MOTOR_BOH, 0);
            }
            else if(/* boh_auto_rot_ == true &&  */
                msg->STICK_L_LR == 0 && 
                msg->STICK_L_UD == 0 && 
                msg->STICK_R_LR == 0)
            {
                //publishCmdToArduino(MOTOR_BOH, uid_(mt_) - boh_random_speed_center_);
                // publishCmdToArduino(MOTOR_BOH, 0);
                is_moving_ = false;

                if(boh_auto_rot_) boh_timer_.start();
                else publishCmdToArduino(MOTOR_BOH, 0);
            }

            //list key -> start or stop rotating boh
            //WARNING: DO NOT PRESS THIS BUTTON WHILE ROTATING THE ROBOT
            if(msg->LIST == PUSHED && prev_listkey_state_ == RELEASED)
            {
                prev_listkey_state_ = PUSHED;

                //if boh is already moving, stops its movement
                //if boh has stopped, starts rotating it
                if(boh_timer_.hasStarted() == true)
                {
                    boh_auto_rot_ = false;
                    publishCmdToArduino(MOTOR_BOH, 0);
                    boh_timer_.stop();
                }
                else
                {
                    boh_auto_rot_ = true;
                    double r = urd_(mt_);
                    int speed = is_moving_ ? 
                        (r*(boh_random_speed_move_max_ - boh_random_speed_move_min_) + 
                            boh_random_speed_move_min_) : 
                        (r*(boh_random_speed_stop_max_ - boh_random_speed_stop_min_) + 
                            boh_random_speed_stop_min_);
                    publishCmdToArduino(MOTOR_BOH, speed);
                    boh_timer_.start();
                }
            }
            if(msg->LIST == RELEASED && prev_listkey_state_ == PUSHED)
            prev_listkey_state_ = RELEASED;
        }

        //Windowsキーで、Seeker用/Hitter用キー配置を切り替える
        if(msg->WINDOWS == PUSHED /* && msg->EIGHT == PUSHED */)
        {
            role_flag_ = (RoleFlag)(!role_flag_);
            if(role_flag_ == HITTER)
            {
                publishCmdToArduino(MOTOR_BOH, 0);
                // publishCmdToArduino(MOTOR_BOH_POS, 0);
                boh_timer_.stop();
            }
            else if(role_flag_ == SEEKER)
            {
                double r = urd_(mt_);
                int speed =  r*(boh_random_speed_stop_max_ - boh_random_speed_stop_min_) + 
                                boh_random_speed_stop_min_;
        
                publishCmdToArduino(MOTOR_BOH, speed);
                boh_timer_.start();
            }
                
            ROS_INFO("ball handler: switched mode to %s", role_flag_ ? "hitter" : "seeker");
        }
    }

    //this function is called to change the rotaion speed randomly
    void timerCb(const ros::TimerEvent &e)
    {
        double r = urd_(mt_);
        int speed = is_moving_ ? 
            (r*(boh_random_speed_move_max_ - boh_random_speed_move_min_) + 
                boh_random_speed_move_min_) : 
            (r*(boh_random_speed_stop_max_ - boh_random_speed_stop_min_) + 
                boh_random_speed_stop_min_);
        
        publishCmdToArduino(MOTOR_BOH, speed);
        // publishCmdToArduino(MOTOR_BOH, 0);
    }

    void startSpin()
    {
        //ros::spin();
        ros::MultiThreadedSpinner spinner(3);
        spinner.spin();
    }

    BallHandler(ros::NodeHandle &nh)
    : dura_(0.5), role_flag_(HITTER) //role_flag_の初期値はHITTER
    {
        double boh_speed_change_period;

        while(!nh.getParam("ball_handler/boh_rot_speed", boh_rot_speed_)){ROS_INFO("error 1"); ROS_ASSERT(false);}
        while(!nh.getParam("ball_handler/boh_autorot_speed", boh_autorot_speed_)){ROS_INFO("error 2"); ROS_ASSERT(false);}
        while(!nh.getParam("ball_handler/boh_random_speed_stop_min", boh_random_speed_stop_min_)){ROS_INFO("error 3"); ROS_ASSERT(false);}
        while(!nh.getParam("ball_handler/boh_random_speed_stop_max", boh_random_speed_stop_max_)){ROS_INFO("error 4"); ROS_ASSERT(false);}
        while(!nh.getParam("ball_handler/boh_random_speed_move_min", boh_random_speed_move_min_)){ROS_INFO("error 5"); ROS_ASSERT(false);}
        while(!nh.getParam("ball_handler/boh_random_speed_move_max", boh_random_speed_move_max_)){ROS_INFO("error 6"); ROS_ASSERT(false);}
        while(!nh.getParam("ball_handler/boh_speed_change_period", boh_speed_change_period)){ROS_INFO("error 7"); ROS_ASSERT(false);}

        ros::SubscribeOptions ops_joy;
        //boost::bind(pointer of member function, reference to instance, args...)
        ops_joy.template initByFullCallbackType<sensor_msgs::Joy::ConstPtr>("joy", 10, boost::bind(&BallHandler::joyCb, this, _1));
        ops_joy.allow_concurrent_callbacks = true;
        cmd_pub_ = nh.advertise<std_msgs::Int16MultiArray>("ball_handler_cmd", 100);
        //joy_sub_ = nh.subscribe("joy", 10, &BallHandler::joyCb, this);
        joy_sub_ = nh.subscribe(ops_joy);
        // omni_sub_ = nh.subscribe("omni_info", 100, &OmniCommander::omniCb, this);

        boh_timer_ = nh.createTimer(ros::Duration(boh_speed_change_period), &BallHandler::timerCb, this);
        boh_timer_.stop();

        cmd_msg_.data.resize(2);

        std::random_device rnd;
        mt_.seed(rnd());
        std::uniform_real_distribution<>::param_type param(0, 1.0);
        urd_.param(param);

        //to prevent boh from rotating freely while the robot is moving,
        //start position control
        // publishCmdToArduino(MOTOR_BOH_POS, 0);
    }
};



int main(int argc, char **argv)
{
    ros::init(argc, argv, "ball_handler");
    ros::NodeHandle nh;
    BallHandler bh(nh);

    bh.startSpin();    
}