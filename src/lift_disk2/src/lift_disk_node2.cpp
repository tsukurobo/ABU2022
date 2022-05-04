//this is a program to drive R2's lifting and catching mechanism

//// Key Bindings (As of 2022/4/26) ////
// 6 -> catch or release a disk, followed by the arm moving upwards or downwards
// 8 -> stop
// 5 -> make the arm horizontal
// 7 -> start homing sequence
// 2 -> move the arm and movable rail upwards
// 3 -> move the arm and movable rail downwards
// up + 2 or 3 -> move the arm upwards or downwards
// down + 2 or 3 -> move the movable rail upwards or downwards

//// Important Specifications of This Program ////
// You can NOT control the arm and the movable rail manually
// while catch or release or homing sequence is being executed.
// Therefore you have to press the stop button 8 first to cancel these sequence
// and are able to control the arm and the rail using button 2 and 3.

// WARNING:
// DO NOT PRESS MULTIPLE BUTTONS SIMULTANEOUSLY. 
// I DO NOT KNOW THE BEHAVIOR OF THIS PROGRAM IN SUCH CASE.

#include <ros/ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <sensor_msgs/Joy.h>
#include "lift_disk2/key_mapping_elecom.h"

class LiftDisk2
{
private:

#pragma region Enums
    enum ButtonState
    {
        RELEASED = 0,
        PUSHED
    };

    enum AxesState
    {
        UP = 1,
        RELEASED_AX = 0,
        DOWN = -1
    };

    enum AirCylinderState
    {
        PUSH = 0,
        PULL,
        NUTRAL
    };

    enum LiftDisk2Actuators
    {
        ARM_MOTOR = 0,
        MOVABLE_RAIL_MOTOR,
        AIR_CYLINDER_HAND,
        AIR_CYLINDER_ROT
    };

    enum ArmHandMotionMode
    {
        CATCH = 0,
        RELEASE
    };

    enum CmdArrayIndex
    {
        ACTUATOR_ID = 0,
        COMMAND
    };
#pragma endregion

#pragma region Private Variables
    ros::Publisher pub_;
    ros::Subscriber joy_sub_, ard_sub_;
    //only one thread can access ard_cmd_ at the same time because of the mutex
    std_msgs::Int16MultiArray ard_cmd_;
    ros::Duration *dura_[2];

    //origin and direction of displacement:
    //arm -> top, downward
    //movable rail -> bottom, upward
    int arm_disp_ = 0,
        movable_rail_disp_ = 0,
        arm_disp_raw_ = 0,
        arm_disp_raw_pre_ = 0,
        movable_rail_disp_raw_ = 0,
        movable_rail_disp_raw_pre_ = 0,
        arm_disp_origin_ = 0,
        movable_rail_disp_origin_ = 0;
    bool is_seeker_mode_on_ = false,
         catch_release_seq_started_ = false,
         homing_seq_started_ = false,
         is_arm_over_max_ = false,
         is_movable_rail_over_max_ = false,
        //  is_arm_under_min_ = false,
         is_movable_rail_under_min_ = false,
         is_stop_button_pressed_ = false,
         arm_touched_ = false,
         arm_touched_pre_ = false,
         movable_rail_touched_ = false,
         movable_rail_touched_pre_ = false,
         first_time_flag_ = true;
    ButtonState prev_state_key2_ = RELEASED,
                prev_state_key3_ = RELEASED;
    ArmHandMotionMode catch_release_seq_mode_ = RELEASE;
    AirCylinderState ac_arm_rot_state_ = PUSH;


    //paramters
    int arm_up_speed_ = 0,
        arm_down_speed_ = 0,
        movable_rail_up_speed_ = 0,
        movable_rail_down_speed_ = 0,
        arm_homing_speed_ = 0,
        movable_rail_homing_speed_ = 0,
        arm_max_limit_ = 0,
        movable_rail_max_limit_ = 0,
        // arm_min_limit_ = 0,
        arm_max_limit_ex_ = 0,
        movable_rail_min_limit_ = 0;

    //this is mutex :)
    std::mutex mtx_, mtx2_;
#pragma endregion

    void publishCmdToArduino(LiftDisk2Actuators id, int val)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        ard_cmd_.data[ACTUATOR_ID] = id;
        ard_cmd_.data[COMMAND] = val;

        pub_.publish(ard_cmd_);
        pub_.publish(ard_cmd_);
    }

    double doubleAbs(double x)
    {
        return (x >= 0) ? x : -x;
    }

public:
    //called when a message from joy_node is received
    void joyCb(const sensor_msgs::Joy &joymsg)
    {
        //7 + 8 -> switch mode
        if(joymsg.SEVEN == PUSHED && joymsg.EIGHT == PUSHED)
        {
            is_seeker_mode_on_ = !is_seeker_mode_on_;
            ROS_INFO("liftdisk_node: seeker mode %s", is_seeker_mode_on_ ? "on" : "off");
            return;
        }

        if(is_seeker_mode_on_ == false)
            return;

        // 2 -> move the arm upwards
        if(joymsg.TWO == PUSHED &&
        catch_release_seq_started_ == false &&
        homing_seq_started_ == false)
        {
            // ROS_INFO("is_arm_under_min = %d\nis_movable_rail_over_max = %d", is_arm_under_min_, is_movable_rail_over_max_);
            prev_state_key2_ = PUSHED;

            // if(is_arm_under_min_ == false && jomsg.AXES_UD != AxesState::DOWN)
            if(arm_touched_ == false && joymsg.AXES_UD != AxesState::DOWN)
            {
                publishCmdToArduino(ARM_MOTOR, arm_up_speed_);
            }

            if(is_movable_rail_over_max_ == false && joymsg.AXES_UD != AxesState::UP)
            {
                publishCmdToArduino(MOVABLE_RAIL_MOTOR, movable_rail_up_speed_);
            }
        }
        // 3 -> move the arm downwards
        else if(joymsg.THREE == PUSHED &&
        catch_release_seq_started_ == false &&
        homing_seq_started_ == false)
        {
            prev_state_key3_ = PUSHED;

            if(is_arm_over_max_ == false && joymsg.AXES_UD != AxesState::DOWN)
            {
                publishCmdToArduino(ARM_MOTOR, arm_down_speed_);
            }

            if(is_movable_rail_under_min_ == false && joymsg.AXES_UD != AxesState::UP)
            // if(movable_rail_touched_ == false && joymsg.AXES_UD != AxesState::UP)
            {
                publishCmdToArduino(MOVABLE_RAIL_MOTOR, movable_rail_down_speed_);
            }
        }

        else if(prev_state_key2_ == PUSHED && joymsg.TWO == ButtonState::RELEASED)
        {
            prev_state_key2_ = ButtonState::RELEASED;
            publishCmdToArduino(ARM_MOTOR, 0);
            publishCmdToArduino(MOVABLE_RAIL_MOTOR, 0);
        }
        else if(prev_state_key3_ == PUSHED && joymsg.THREE == ButtonState::RELEASED)
        {
            prev_state_key3_ = ButtonState::RELEASED;
            publishCmdToArduino(ARM_MOTOR, 0);
            publishCmdToArduino(MOVABLE_RAIL_MOTOR, 0);
        }

        // 6 -> catch or release a disk, following the arm moving upwards or downwards
        if(joymsg.SIX == PUSHED && 
        catch_release_seq_started_ == false &&
        homing_seq_started_ == false)
        {
            ROS_INFO("liftdisk_node: %s sequence has started", 
                (catch_release_seq_mode_) == CATCH ? "catch" : "release");

            //set the flag
            catch_release_seq_started_ = true;

            //drive the air cylinder
            ROS_INFO("liftdisk_node: driving air cylinder...");
            publishCmdToArduino(AIR_CYLINDER_HAND, 
                (catch_release_seq_mode_ == CATCH) ? PUSH : PULL);

            //wait a few moments
            dura_[catch_release_seq_mode_]->sleep();

            //move the arm and movable rail upwards (after catching a disk)
            //or downwards (after releasing a disk)

            ROS_INFO("liftdisk_node: moving the arm and the rail %s ", 
                (catch_release_seq_mode_) == CATCH ? "upwards" : "downwards");
            
            // if((catch_release_seq_mode_ == CATCH && is_arm_under_min_ == true) ||
            //     (catch_release_seq_mode_ == RELEASE && is_arm_over_max_ == true))
            if((catch_release_seq_mode_ == CATCH && arm_touched_ == true) ||
                (catch_release_seq_mode_ == RELEASE && is_arm_over_max_ == true))
            {
                publishCmdToArduino(ARM_MOTOR, 0);
                ROS_INFO("liftdisk_node: cannot move the arm because its displacement is out of the limit");
            }
            else
            {
                publishCmdToArduino(ARM_MOTOR, 
                    (catch_release_seq_mode_ == CATCH) ? arm_up_speed_ : arm_down_speed_);
            }

            if(ac_arm_rot_state_ == PUSH)
            {
                // if((catch_release_seq_mode_ == CATCH && is_movable_rail_over_max_ == true) ||
                //     (catch_release_seq_mode_ == RELEASE && is_movable_rail_under_min_ == true))
                if(is_movable_rail_under_min_ == true)
                {
                    publishCmdToArduino(MOVABLE_RAIL_MOTOR, 0);
                    ROS_INFO("liftdisk_node: cannot move the movable_rail because its displacement is out of the limit");
                }
                else
                {
                    publishCmdToArduino(MOVABLE_RAIL_MOTOR, movable_rail_down_speed_);
                }
            }
            else if(ac_arm_rot_state_ == PULL)
            {
                if((catch_release_seq_mode_ == CATCH && is_movable_rail_over_max_ == true) ||
                    (catch_release_seq_mode_ == RELEASE && is_movable_rail_under_min_ == true))
                {
                    publishCmdToArduino(MOVABLE_RAIL_MOTOR, 0);
                    ROS_INFO("liftdisk_node: cannot move the movable_rail because its displacement is out of the limit");
                }
                else
                {
                    publishCmdToArduino(MOVABLE_RAIL_MOTOR, 
                        (catch_release_seq_mode_ == CATCH) ? movable_rail_up_speed_ : movable_rail_down_speed_);
                }
            }

            //wait until both the arm and the rail reaches the limit of their movable range
            //or this program is shutdowned or the stop button is pressed
            char seq_end_flag = 0;
            while(ros::ok())
            {
                if(arm_touched_ == true)
                    seq_end_flag |= 0b00000001;
                // if(movable_rail_touched_ == true)
                //     seq_end_flag |= 0b00000010;

                // if(catch_release_seq_mode_ == CATCH && 
                // is_arm_under_min_ == true && 
                // is_movable_rail_over_max_ == true)
                if(catch_release_seq_mode_ == CATCH &&
                ac_arm_rot_state_ == PUSH && 
                seq_end_flag == 0b0000001 &&
                is_movable_rail_under_min_ == true)
                {
                    break;
                }
                else if(catch_release_seq_mode_ == CATCH &&
                ac_arm_rot_state_ == PULL && 
                seq_end_flag&0b00000001 == 1 && 
                is_movable_rail_over_max_ == true)
                {
                    break;
                }
                // else if(catch_release_seq_mode_ == RELEASE &&
                // is_arm_over_max_ == true &&
                // is_movable_rail_under_min_ == true)
                else if(catch_release_seq_mode_ == RELEASE &&
                is_arm_over_max_ == true &&
                /*seq_end_flag&0b00000010 == 2*/
                is_movable_rail_under_min_ == true)
                {
                    break;
                }
                else if(is_stop_button_pressed_ == true)
                {
                    is_stop_button_pressed_ = false;
                    publishCmdToArduino(ARM_MOTOR, 0);
                    publishCmdToArduino(MOVABLE_RAIL_MOTOR, 0);
                    break;
                }
            }
            ROS_INFO("liftdisk_node: %s sequence has finished", 
                (catch_release_seq_mode_) == CATCH ? "catch" : "release");

            //switch catch and release mode alternately
            catch_release_seq_mode_ = (ArmHandMotionMode)(1-catch_release_seq_mode_);
            catch_release_seq_started_ = false;
            
            return;
        }

        // 7 -> start homing sequence
        if(joymsg.SEVEN == PUSHED &&
        catch_release_seq_started_ == false &&
        homing_seq_started_ == false)
        {
            char homing_end_flag = 0;

            ROS_INFO("liftdisk_node: homing sequence has started");
            
            //set the flag
            homing_seq_started_ = true;

            //drive motors
            if(arm_touched_ == true)
            {
                publishCmdToArduino(ARM_MOTOR, 0);
                ROS_INFO("liftdisk_node: cannot move the arm because it already touched to the top switch");
            }
            else
            {
                publishCmdToArduino(ARM_MOTOR, arm_homing_speed_);
            }
            if(movable_rail_touched_ == true)
            {
                publishCmdToArduino(MOVABLE_RAIL_MOTOR, 0);
                ROS_INFO("liftdisk_node: cannot move the movable rail because it already touched to the bottom switch");
            }
            else
            {
                publishCmdToArduino(MOVABLE_RAIL_MOTOR, movable_rail_homing_speed_);
            }
            
            //wait until both the arm and the rail reaches the their home positions
            //or this program is shutdowned or the stop button is pressed
            while(ros::ok())
            {
                if(arm_touched_ == true)
                    homing_end_flag |= 0b00000001;
                if(movable_rail_touched_ == true)
                    homing_end_flag |= 0b00000010;

                if(homing_end_flag == 0b00000011)
                {
                    //set origin of displacement
                    arm_disp_origin_ = arm_disp_raw_;
                    movable_rail_disp_origin_ = movable_rail_disp_raw_;
                    break;
                }
                if(is_stop_button_pressed_ == true)
                {
                    is_stop_button_pressed_ = false;
                    publishCmdToArduino(ARM_MOTOR, 0);
                    publishCmdToArduino(MOVABLE_RAIL_MOTOR, 0);
                    break;
                }
            }
            ROS_INFO("liftdisk_node: homing sequence has finished");
            
            //clear the flag
            homing_seq_started_ = false;

            {
                std::lock_guard<std::mutex> lock(mtx2_);
                is_arm_over_max_ = false;
                // is_arm_under_min_ = false;
                is_movable_rail_over_max_ = false;
                // is_movable_rail_under_min_ = false;
            }

            return;
        }

        // 5 -> make the arm horizontal
        if(joymsg.FIVE == PUSHED)
        {
            //set the bottom limit of the movable rail according with the arm state
            if(ac_arm_rot_state_ == PUSH)
                arm_max_limit_ += arm_max_limit_ex_;
            else if(ac_arm_rot_state_ == PULL)
                arm_max_limit_ -= arm_max_limit_ex_;
            //actually the arm can not be vertical by air cylinder once it is set horizontal
            publishCmdToArduino(AIR_CYLINDER_ROT, ac_arm_rot_state_);
            ac_arm_rot_state_ = (AirCylinderState)(1 - ac_arm_rot_state_); 

            // dura_ac_->sleep();
            // publishCmdToArduino(AIR_CYLINDER_ROT, NUTRAL);
        }

        // 8 -> stop
        if(joymsg.EIGHT == PUSHED)
        {
            if(homing_seq_started_ || catch_release_seq_started_)
                is_stop_button_pressed_ = true;
        }
    }

    void arduinoCb(const std_msgs::Int32MultiArray::ConstPtr &data)
    {
        if(is_seeker_mode_on_ == false)
            return;

        if(first_time_flag_)
        {
            arm_disp_raw_pre_ = data->data[0];
            movable_rail_disp_raw_pre_ = data->data[1];
            first_time_flag_ = false;
        }   

        if(doubleAbs(data->data[0] - arm_disp_raw_pre_) < 50)
            arm_disp_raw_ = data->data[0];
        else
            arm_disp_raw_ = arm_disp_raw_pre_;
        arm_disp_raw_pre_ = arm_disp_raw_;

        if(doubleAbs(data->data[1] - movable_rail_disp_raw_pre_) < 50)
            movable_rail_disp_raw_ = data->data[1];
        else
            movable_rail_disp_raw_ = movable_rail_disp_raw_pre_;
        movable_rail_disp_raw_pre_ = movable_rail_disp_raw_;

        // movable_rail_disp_raw_ = data->data[1];
        arm_disp_ = arm_disp_raw_ - arm_disp_origin_;
        movable_rail_disp_ = movable_rail_disp_raw_ - movable_rail_disp_origin_;
        arm_touched_ = data->data[2];
        movable_rail_touched_ = data->data[3];
        ROS_INFO("arm_disp = %d, movable_rail_disp = %d", arm_disp_, movable_rail_disp_);
        ROS_INFO("is_movable_rail_over_max = %d, is_arm_over_max = %d", is_movable_rail_over_max_, is_arm_over_max_);
        //stop the arm if its displacement is over or under the limit or the switch is pressed
        // if(homing_seq_started_ == false)
        // {
            if(arm_disp_ >= arm_max_limit_ && is_arm_over_max_ == false)
            {
                ROS_INFO("liftdisk_node: arm is over the maximun limit");
                {
                    std::lock_guard<std::mutex> lock(mtx2_);
                    is_arm_over_max_ = true;
                    // is_arm_under_min_ = false;
                }
                publishCmdToArduino(ARM_MOTOR, 0);
            }
            // else if(arm_disp_ <= arm_min_limit_ && is_arm_under_min_ == false)
            // {
            //     ROS_INFO("liftdisk_node: arm is under the minimun limit");
            //     {
            //         std::lock_guard<std::mutex> lock(mtx2_);
            //         is_arm_under_min_ = true;
            //         is_arm_over_max_ = false;
            //     }
            //     publishCmdToArduino(ARM_MOTOR, 0);
            // }
            else if(/*arm_min_limit_ < arm_disp_ && */arm_disp_ < arm_max_limit_)
            {
                std::lock_guard<std::mutex> lock(mtx2_);
                is_arm_over_max_ = false;
                // is_arm_under_min_ = false;
            }

            //stop the movable rail if its displacement is over or under the limit
            if(movable_rail_disp_ >= movable_rail_max_limit_ &&
            is_movable_rail_over_max_ == false)
            {
                ROS_INFO("liftdisk_node: movable rail is over the maximun limit");
                {
                    std::lock_guard<std::mutex> lock(mtx2_);
                    is_movable_rail_over_max_ = true;
                    // is_movable_rail_under_min_ = false;
                }
                publishCmdToArduino(MOVABLE_RAIL_MOTOR, 0);
            }
            else if(movable_rail_disp_ <= movable_rail_min_limit_ &&
            is_movable_rail_under_min_ == false && 
            homing_seq_started_ == false)
            {
                ROS_INFO("liftdisk_node: movable rail is under the minimun limit");
                {
                    std::lock_guard<std::mutex> lock(mtx2_);
                    is_movable_rail_under_min_ = true;
                    is_movable_rail_over_max_ = false;
                }
                publishCmdToArduino(MOVABLE_RAIL_MOTOR, 0);
            }
            else if(movable_rail_min_limit_ < movable_rail_disp_ &&
            movable_rail_disp_ < movable_rail_max_limit_)
            {
                std::lock_guard<std::mutex> lock(mtx2_);
                is_movable_rail_over_max_ = false;
                is_movable_rail_under_min_ = false;
            }
        // }
        // else if(homing_seq_started_ == true)
        // {
            if(arm_touched_ == true && arm_touched_pre_ == false)
            {
                ROS_INFO("liftdisk_node: arm has contacted with the top switch");
                publishCmdToArduino(ARM_MOTOR, 0);
            }
            if(movable_rail_touched_ == true && movable_rail_touched_pre_ == false)
            {
                ROS_INFO("liftdisk_node: movable rail has contacted with the bottom switch");
                publishCmdToArduino(MOVABLE_RAIL_MOTOR, 0);
            }
        // }
        arm_touched_pre_ = arm_touched_;
        movable_rail_touched_pre_ = movable_rail_touched_;
    }

    LiftDisk2(ros::NodeHandle &nh)
    {
        //initialize publisher and subscriber
        pub_ = nh.advertise<std_msgs::Int16MultiArray>("liftdisk2_cmd", 100);

        ros::SubscribeOptions ops_joy;
        //boost::bind(pointer of member function, reference or pointer of the instance, args...)
        ops_joy.template initByFullCallbackType<sensor_msgs::Joy>("joy", 100, boost::bind(&LiftDisk2::joyCb, this, _1));
        ops_joy.allow_concurrent_callbacks = true;
        //joy_sub_ = nh.subscribe("joy", 100, &Shooter2::joyCb, this);
        joy_sub_ = nh.subscribe(ops_joy);
        ard_sub_ = nh.subscribe("liftdisk2_data", 100, &LiftDisk2::arduinoCb, this);

        ard_cmd_.data.resize(2);
        
        //configure parameters
        while(!nh.getParam("arm/up_speed", arm_up_speed_)){ROS_ASSERT("error0"); ROS_ASSERT(false);}
        while(!nh.getParam("arm/down_speed", arm_down_speed_)){ROS_INFO("error1"); ROS_ASSERT(false);}
        while(!nh.getParam("arm/homing_speed", arm_homing_speed_)){ROS_INFO("error2"); ROS_ASSERT(false);}
        while(!nh.getParam("arm/max_limit", arm_max_limit_)){ROS_INFO("error3"); ROS_ASSERT(false);}
        // while(!nh.getParam("arm/min_limit", arm_min_limit_)){ROS_INFO("error4"); ROS_ASSERT(false);}
        while(!nh.getParam("arm/max_limit_ex", arm_max_limit_ex_)){ROS_INFO("error10"); ROS_ASSERT(false);}
        while(!nh.getParam("movable_rail/up_speed", movable_rail_up_speed_)){ROS_INFO("error5"); ROS_ASSERT(false);}
        while(!nh.getParam("movable_rail/down_speed", movable_rail_down_speed_)){ROS_INFO("error6"); ROS_ASSERT(false);}
        while(!nh.getParam("movable_rail/homing_speed", movable_rail_homing_speed_)){ROS_INFO("error7"); ROS_ASSERT(false);}
        while(!nh.getParam("movable_rail/max_limit", movable_rail_max_limit_)){ROS_INFO("error8"); ROS_ASSERT(false);}
        while(!nh.getParam("movable_rail/min_limit", movable_rail_min_limit_)){ROS_INFO("error9"); ROS_ASSERT(false);}

        double sleep_t_after_catch = 0,
               sleep_t_after_release = 0;
        while(!nh.getParam("others/sleep_t_after_catch", sleep_t_after_catch)){ROS_INFO("error8");}
        while(!nh.getParam("others/sleep_t_after_release", sleep_t_after_release)){ROS_INFO("error9");}
        
        dura_[0] = new ros::Duration(sleep_t_after_catch);
        dura_[1] = new ros::Duration(sleep_t_after_release);
    }

    ~LiftDisk2()
    {
        delete dura_[0];
        delete dura_[1];
    }

    void startSpin()
    {
        ros::MultiThreadedSpinner spinner(3);
        spinner.spin();
    }

};

int main(int argc, char **argv)
{
    //ROSノードの初期化
    ros::init(argc, argv, "liftdisk2_node");
    ros::NodeHandle nh;

    LiftDisk2 liftdisk2(nh);

    liftdisk2.startSpin();

}