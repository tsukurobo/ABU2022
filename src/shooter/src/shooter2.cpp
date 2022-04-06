//R1射出・装填・照準・昇降機構制御用ROS側プログラム(2号機用)
#include <ros/ros.h>
#include "shooter/key_mapping_elecom.h"
#include <abu2022_msgs/R1ArduinoCmd.h>
//#include <abu2022_msgs/pwm.h>
#include <abu2022_msgs/R1ArduinoData.h>
#include <sensor_msgs/Joy.h>
#include <mutex>
//#define PUSHED 1

class Shooter2
{
private:
    enum ButtonState {PUSHED=1};
    enum Side
    {
        RIGHT = 0,
        LEFT
    };
    enum ServoState
    {
        WIPE=0,
        RESET
    };
    enum RollerMotorState
    {
        ROTATE = 0,
        STOP
    };
    enum LiftMotorState
    {
        BOTTOM = 0,
        MIDDLE,
        TOP
    };
    enum LoadMotorState
    {
        LOAD = 0,
        INIT_POS
    };
    enum AirCylinderState
    {
        PUSH = 0,
        PULL,
        NUTRAL
    };
    enum Shooter2Actuator
    {
        PointerMotor = 0,
        RollerUpDownMotor,
        RollerMotorR,
        RollerMotorL,
        LiftMotor,
        LiftMotorNoPID,
        ServoR,
        ServoL,
        LoadMotor,
        AirCylinder
    };

    //排他制御を適用し、複数のスレッドが同時にard_cmd_にアクセスできないようにする
    ros::Publisher pub_/*cubic_pub_*/;
    ros::Subscriber joy_sub_, ard_sub_;
    abu2022_msgs::R1ArduinoCmd ard_cmd_;
    // abu2022_msgs::pwm cubic_cmd_;
    ros::Duration *dura_, *dura2_, *dura_short_;
    int load_motor_angle_ = 0;
    int lift_motor_angle_ = 0;
    int lift_motor_angle_origin_ = 0;
    bool lift_motor_sw_ = false;
    //各種シーケンス管理用フラグ
    bool is_shoot_seq_started_ = false,
         is_homing_seq_started_ = false,
         is_stop_mode_enabled_ = false,
         is_load_seq_l_started_ = false,
         is_load_seq_c_started_ = false,
         is_load_seq_r_started_ = false,
         is_lifting_seq_started_ = false,
         is_lagori_break_seq_started_ = false;
    int lagori_break_count_ = 0;

    //パラメータ
    int roller_rot_speed_ = 0;
    int roller_updown_speed_ = 0;
    int servo_angles_[2][2] = {0};
    int pointer_rot_speed_ = 0;
    //角度0 = 初期位置 = Arduinoの電源を入れた時のモーターの回転角度
    int lift_motor_angle_goal_[3] = {0};
    int load_motor_angle_goal_[2] = {0};
    int lift_motor_updown_speed_ = 0;
    double sleep_t_ = 1.0;
    
    //キーを押すたびに動作が変わるようにするための変数(0 or 1)
    //ServoState servo_angle_selector_[2] = {ServoState::RESET};
    RollerMotorState roller_motor_state_ = RollerMotorState::STOP;

    //ミューテックス
    std::mutex mtx_;

    //ちょっとした関数たち
    //th > 0と仮定
    double step(double x, double th)
    {
        if(x >= th)
            return 1.0;
        else if(x <= -th)
            return -1.0;
        else
            return 0;
    }
    
    //range > 0と仮定
    bool isInRange(double x, double center, double range)
    {
        return (center-range <= x && x <= center+range);
    }

    void publishCmdToArduino(Shooter2Actuator id, int val)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        ard_cmd_.actuator_id = id;
        ard_cmd_.value = val;

        pub_.publish(ard_cmd_);
        dura_short_->sleep();
    }

    // void publishCmdToCubic(int rot_flag, int freq, int duty)
    // {
    //     cubic_cmd_.on_off = rot_flag;
    //     cubic_cmd_.freq = freq;
    //     cubic_cmd_.duty = duty;

    //     cubic_pub_.publish(cubic_cmd_);
    // }

public:
    //ジョイコンからのデータが来たときに呼ばれる関数
    void joyCb(const sensor_msgs::Joy &joymsg)
    {
        if(is_stop_mode_enabled_ == false)
        {
            //右スティク左右で照準機構を回転させる
            //階段関数を使用して操作性の改善を図る
            publishCmdToArduino(Shooter2Actuator::PointerMotor,
               pointer_rot_speed_*step(joymsg.STICK_R_LR, 0.5));

            //右スティック上下でローラー機構の上下方向の向きを変化させる
            publishCmdToArduino(Shooter2Actuator::RollerUpDownMotor,
               roller_updown_speed_*step(joymsg.STICK_R_UD, 0.5));

            //12キーでローラーの回転・停止
            if(joymsg.TWELVE == ButtonState::PUSHED)
            {
                publishCmdToArduino(Shooter2Actuator::RollerMotorR,
                    (roller_motor_state_ == RollerMotorState::STOP) ? roller_rot_speed_ : 0);
                // publishCmdToCubic(
                //     (roller_motor_state_ == RollerMotorState::STOP) ? 1 : 0, 20, roller_rot_speed_);

                publishCmdToArduino(Shooter2Actuator::RollerMotorL,
                    (roller_motor_state_ == RollerMotorState::STOP) ? -roller_rot_speed_ : 0);

                roller_motor_state_ = static_cast<RollerMotorState>(1 - roller_motor_state_);
            }

            //1キーで装填シーケンス1(左サーボからの装填)作動
            if(joymsg.ONE == ButtonState::PUSHED && 
                is_load_seq_l_started_ == false && 
                is_load_seq_c_started_ == false && 
                is_load_seq_r_started_ == false &&
                is_lagori_break_seq_started_ == false)
            {
                ROS_INFO("shooter2: load sequence(L) has started");
                is_load_seq_l_started_ = true;
                
                //左サーボを動作させる
                ROS_INFO("shooter2: driving servo (L)...");
                publishCmdToArduino(Shooter2Actuator::ServoL, servo_angles_[Side::LEFT][ServoState::WIPE]);
                dura_->sleep();
                publishCmdToArduino(Shooter2Actuator::ServoL, servo_angles_[Side::LEFT][ServoState::RESET]);
                dura2_->sleep();

                //装填用DCモータを回転させる
                ROS_INFO("shooter2: driving dc motor...");
                publishCmdToArduino(Shooter2Actuator::LoadMotor, load_motor_angle_goal_[LoadMotorState::LOAD]);
                //回転が完了するまで待つ
                // while(isInRange((double)load_motor_angle_*0.1, load_motor_angle_goal_[LoadMotorState::LOAD], 5.0) == false && ros::ok())
                // {
                //     //ROS_INFO("%f", load_motor_angle_*0.1-load_motor_angle_goal_[LoadMotorState::LOAD]);
                //     //停止スイッチが押された場合はモーターを停止させる
                //     if(is_stop_mode_enabled_ || !ros::ok())
                //     {
                //         publishCmdToArduino(Shooter2Actuator::LoadMotor, load_motor_angle_*0.1);
                //         is_load_seq_l_started_ = false;
                //         return;
                //     }
                // }
                //ros::Duration(1.0).sleep();

                //少し待つ
                dura_->sleep();
                //DCをもとの位置に戻す
                ROS_INFO("shooter2: resetting...");
                publishCmdToArduino(Shooter2Actuator::LoadMotor, load_motor_angle_goal_[LoadMotorState::INIT_POS]);

                //エアシリ作動
                /*ROS_INFO("shooter2: driving air cylinder...");
                publishCmdToArduino(Shooter2Actuator::AirCylinder, AirCylinderState::PUSH);
                //少し待つ
                dura_->sleep();
                //エアシリのロッドを引っ込め、DCとサーボをもとの位置に戻す
                ROS_INFO("shooter2: resetting...");
                publishCmdToArduino(Shooter2Actuator::AirCylinder, AirCylinderState::PULL);
                dura_->sleep();
                publishCmdToArduino(Shooter2Actuator::AirCylinder, AirCylinderState::NUTRAL);
                publishCmdToArduino(Shooter2Actuator::LoadMotor, load_motor_angle_goal_[LoadMotorState::INIT_POS]);*/

                is_load_seq_l_started_ = false;
                return;
            }

            //4キーで装填シーケンス2(右サーボからの装填)作動
            if(joymsg.FOUR == ButtonState::PUSHED &&
                is_load_seq_l_started_ == false &&
                is_load_seq_c_started_ == false &&
                is_load_seq_r_started_ == false &&
                is_lagori_break_seq_started_ == false)
            {
                ROS_INFO("shooter2: load sequence(R) has started");
                is_load_seq_r_started_ = true;
                
                //右サーボを動作させる
                ROS_INFO("shooter2: driving servo (R)...");
                publishCmdToArduino(Shooter2Actuator::ServoR, servo_angles_[Side::RIGHT][ServoState::WIPE]);
                dura_->sleep();
                publishCmdToArduino(Shooter2Actuator::ServoR, servo_angles_[Side::RIGHT][ServoState::RESET]);
                dura2_->sleep();

                //装填用DCモータを回転させる
                ROS_INFO("shooter2: driving dc motor...");
                publishCmdToArduino(Shooter2Actuator::LoadMotor, load_motor_angle_goal_[LoadMotorState::LOAD]);
                // //回転が完了するまで待つ
                // while(isInRange((double)load_motor_angle_*0.1, load_motor_angle_goal_[LoadMotorState::LOAD], 5.0) == false && ros::ok())
                // {
                //     //ROS_INFO("%f", load_motor_angle_*0.1-load_motor_angle_goal_[LoadMotorState::LOAD]);
                //     //停止スイッチが押された場合はモーターを停止させる
                //     if(is_stop_mode_enabled_ || !ros::ok())
                //     {
                //         publishCmdToArduino(Shooter2Actuator::LoadMotor, load_motor_angle_*0.1);
                //         is_load_seq_r_started_ = false;
                //         return;
                //     }
                // }
                //ros::Duration(1.0).sleep();

                //少し待つ
                dura_->sleep();
                //DCをもとの位置に戻す
                ROS_INFO("shooter2: resetting...");
                publishCmdToArduino(Shooter2Actuator::LoadMotor, load_motor_angle_goal_[LoadMotorState::INIT_POS]);

                is_load_seq_r_started_ = false;
                return;
            }

            //2キーで装填シーケンス3(装填モーターのみ)作動
            if(joymsg.TWO == ButtonState::PUSHED &&
                is_load_seq_l_started_ == false &&
                is_load_seq_c_started_ == false &&
                is_load_seq_r_started_ == false &&
                is_lagori_break_seq_started_ == false)
            {
                ROS_INFO("shooter2: load sequence(C) has started");
                is_load_seq_c_started_ = true;

                //装填用DCモータを回転させる
                ROS_INFO("shooter2: driving dc motor...");
                publishCmdToArduino(Shooter2Actuator::LoadMotor, load_motor_angle_goal_[LoadMotorState::LOAD]);
                //回転が完了するまで待つ
                // while(isInRange((double)load_motor_angle_*0.1, load_motor_angle_goal_[LoadMotorState::LOAD], 5.0) == false && ros::ok())
                // {
                //     //ROS_INFO("%f", load_motor_angle_*0.1-load_motor_angle_goal_[LoadMotorState::LOAD]);
                //     //停止スイッチが押された場合はモーターを停止させる
                //     if(is_stop_mode_enabled_ || !ros::ok())
                //     {
                //         publishCmdToArduino(Shooter2Actuator::LoadMotor, load_motor_angle_*0.1);
                //         is_load_seq_c_started_ = false;
                //         return;
                //     }
                // }
                //ros::Duration(1.0).sleep();

                //少し待つ
                dura_->sleep();
                //DCをもとの位置に戻す
                ROS_INFO("shooter2: resetting...");
                publishCmdToArduino(Shooter2Actuator::LoadMotor, load_motor_angle_goal_[LoadMotorState::INIT_POS]);

                is_load_seq_c_started_ = false;
                return;
            }

            //6キーで射出シーケンス作動
            if(joymsg.SIX == ButtonState::PUSHED &&
                is_shoot_seq_started_ == false &&
                is_lagori_break_seq_started_ == false)
            {
                ROS_INFO("shooter2: shoot sequence has started");
                is_shoot_seq_started_ = true;

                //エアシリを動作させる
                ROS_INFO("shooter2: driving air cylinder...");
                publishCmdToArduino(Shooter2Actuator::AirCylinder, AirCylinderState::PUSH);
                //少し待つ
                dura2_->sleep();
                //エアシリのロッドを引っ込める
                ROS_INFO("shooter2: resetting...");
                publishCmdToArduino(Shooter2Actuator::AirCylinder, AirCylinderState::PULL);

                is_shoot_seq_started_ = false;
                return;
            }

            // //十字キー上下で昇降機構を上昇させる
            // if(is_homing_seq_started_ == false)
            //     publishCmdToArduino(Shooter2Actuator::LiftMotor, -lift_motor_updown_speed_*joymsg.AXES_UD);

            //十字キー下で原点位置まで移動
            if(joymsg.AXES_UD == -ButtonState::PUSHED && 
                is_homing_seq_started_ == false &&
                is_lifting_seq_started_ == false)
            {
                ROS_INFO("shooter2: homing sequence has started");
                is_homing_seq_started_ = true;
                publishCmdToArduino(Shooter2Actuator::LiftMotorNoPID, 200);
                while(lift_motor_sw_ == false)
                {
                    if(is_stop_mode_enabled_ || !ros::ok())
                    {
                        is_homing_seq_started_ = false;
                        publishCmdToArduino(Shooter2Actuator::LiftMotorNoPID, 0);
                        return;
                    }
                }
                //ros::Duration(1.0).sleep();

                publishCmdToArduino(Shooter2Actuator::LiftMotorNoPID, 0);
                lift_motor_angle_origin_ = lift_motor_angle_*0.1;
                is_homing_seq_started_ = false;
                ROS_INFO("shooter2: homing sequence has finished");
                return;
            }

            //十字キー上で昇降機構を上昇させる
            if(joymsg.AXES_UD == ButtonState::PUSHED && is_lifting_seq_started_ == false &&
                is_homing_seq_started_ == false)
            {
                double goal_angle = 0;
                ROS_INFO("shooter2: lift sequence (up) has started");
                is_lifting_seq_started_ = true;

                if(joymsg.FIVE == ButtonState::PUSHED)
                    goal_angle = lift_motor_angle_origin_ + lift_motor_angle_goal_[LiftMotorState::TOP];
                else
                    goal_angle = lift_motor_angle_origin_ + lift_motor_angle_goal_[LiftMotorState::MIDDLE];

                publishCmdToArduino(Shooter2Actuator::LiftMotor, goal_angle);
                //回転が完了するまで待つ
                while(isInRange((double)lift_motor_angle_*0.1, goal_angle, 10.0) == false && ros::ok())
                {
                    //ROS_INFO("%f", load_motor_angle_*0.1-load_motor_angle_goal_[LoadMotorState::LOAD]);
                    //停止スイッチが押された場合はモーターを停止させる
                    if(is_stop_mode_enabled_ || !ros::ok())
                    {
                        publishCmdToArduino(Shooter2Actuator::LiftMotorNoPID, 0);
                        is_lifting_seq_started_ = false;
                        return;
                    }
                }
                //ros::Duration(1.0).sleep();

                ROS_INFO("shooter2: lift sequence (up) has finished");
                is_lifting_seq_started_ = false;
                return;
            }
            
            //左ジョイスティック上下で昇降機構を手動操作
            if(is_lifting_seq_started_ == false &&is_homing_seq_started_ == false)
            {
                publishCmdToArduino(Shooter2Actuator::LiftMotorNoPID,
                    -lift_motor_updown_speed_*step(joymsg.STICK_L_UD, 0.5));
            }

            // //十字キー下で昇降機構を下降させる
            // if(joymsg.AXES_UD == -ButtonState::PUSHED)
            // {
            //     //ROS_INFO("shooter2: lift sequence (down) has started");

            //     publishCmdToArduino(Shooter2Actuator::LiftMotor, lift_motor_angle_goal_[LiftMotorState::BOTTOM]);
            // }

            // //十字キー左で、正面から見て右側のサーボモータを作動させる
            // if(joymsg.AXES_LR == ButtonState::PUSHED)
            // {
            //     publishCmdToArduino(Shooter2Actuator::ServoR, servo_angles_[Side::RIGHT][servo_angle_selector_[Side::RIGHT]]);
            //     servo_angle_selector_[Side::RIGHT] = static_cast<ServoState>(1-servo_angle_selector_[Side::RIGHT]);
            // }

            // //十字キー右で、正面から見て左側のサーボモータを作動させる
            // if(joymsg.AXES_LR == -ButtonState::PUSHED)
            // {
            //     publishCmdToArduino(Shooter2Actuator::ServoL, servo_angles_[Side::LEFT][servo_angle_selector_[Side::LEFT]]);
            //     servo_angle_selector_[Side::LEFT] = static_cast<ServoState>(1-servo_angle_selector_[Side::LEFT]);
            // }

            //3キーでラゴリブレイク用シーケンス作動
            if(joymsg.THREE == ButtonState::PUSHED && 
                is_load_seq_l_started_ == false &&
                is_load_seq_c_started_ == false &&
                is_load_seq_r_started_ == false &&
                is_shoot_seq_started_ == false &&
                is_lagori_break_seq_started_ == false)
            {
                //サーボを動かす
                ROS_INFO("shooter2: lagori break seq has started");
                is_lagori_break_seq_started_ = true;
                
                if(lagori_break_count_ == 2 || lagori_break_count_ == 1)
                {
                    if(lagori_break_count_ == 2)
                    {
                        //右サーボを動作させる
                        ROS_INFO("shooter2: driving servo (R)...");
                        publishCmdToArduino(Shooter2Actuator::ServoR, servo_angles_[Side::RIGHT][ServoState::WIPE]);
                        dura_->sleep();
                        publishCmdToArduino(Shooter2Actuator::ServoR, servo_angles_[Side::RIGHT][ServoState::RESET]);
                        dura2_->sleep();
                    }

                    //装填用DCモータを回転させる
                    ROS_INFO("shooter2: driving dc motor...");
                    publishCmdToArduino(Shooter2Actuator::LoadMotor, load_motor_angle_goal_[LoadMotorState::LOAD]);
                    //少し待つ
                    dura_->sleep();
                    //DCをもとの位置に戻す
                    ROS_INFO("shooter2: resetting dc motor...");
                    publishCmdToArduino(Shooter2Actuator::LoadMotor, load_motor_angle_goal_[LoadMotorState::INIT_POS]);
                    dura2_->sleep();
                }

                ROS_INFO("shooter2: shoot sequence has started");
                //エアシリを動作させる
                ROS_INFO("shooter2: driving air cylinder...");
                publishCmdToArduino(Shooter2Actuator::AirCylinder, AirCylinderState::PUSH);
                //少し待つ
                dura2_->sleep();
                //エアシリのロッドを引っ込める
                ROS_INFO("shooter2: resetting air cylinder...");
                publishCmdToArduino(Shooter2Actuator::AirCylinder, AirCylinderState::PULL);

                is_lagori_break_seq_started_ = false;
                lagori_break_count_++;
                if(lagori_break_count_ == 3)
                    lagori_break_count_ = 0;

                return;
            }
        }
        
        //7キーでモーター及び各シーケンスの停止/作動の切り替え
        if(joymsg.SEVEN == ButtonState::PUSHED)
        {
            if(is_stop_mode_enabled_ == false)
            {
                ROS_INFO("shooter2: stop mode enabled");
                is_stop_mode_enabled_ = true;
                publishCmdToArduino(Shooter2Actuator::RollerMotorR, 0);
                publishCmdToArduino(Shooter2Actuator::RollerMotorL, 0);
                publishCmdToArduino(Shooter2Actuator::LiftMotorNoPID, 0);
            }
            else if(is_stop_mode_enabled_ == true)
            {
                ROS_INFO("shooter2: stop mode disabled");
                is_stop_mode_enabled_ = false;
            }
        }
    }

    void arduinoCb(const abu2022_msgs::R1ArduinoData::ConstPtr &data)
    {
        //単位はどちらもx0.1 deg
        load_motor_angle_ = data->load_motor_angle;
        lift_motor_angle_ = data->lift_motor_angle;
        lift_motor_sw_ = data->lift_motor_sw;
    }

    Shooter2(ros::NodeHandle &nh)
    {
        //パブリッシャ・サブスクライバの初期化
        pub_ = nh.advertise<abu2022_msgs::R1ArduinoCmd>("arduino_cmd", 100);
        // cubic_pub_ = nh.advertise<abu2022_msgs::pwm>("pwm", 100);

        ros::SubscribeOptions ops_joy;
        //boost::bind(メンバ関数ポインタ, インスタンスの参照orポインタ, args...)
        ops_joy.template initByFullCallbackType<sensor_msgs::Joy>("joy", 100, boost::bind(&Shooter2::joyCb, this, _1));
        ops_joy.allow_concurrent_callbacks = true;
        //joy_sub_ = nh.subscribe("joy", 100, &Shooter2::joyCb, this);
        joy_sub_ = nh.subscribe(ops_joy);
        ard_sub_ = nh.subscribe("arduino_data", 100, &Shooter2::arduinoCb, this);
        
        //パラメータの設定
        while(!nh.getParam("pointer/rotation_speed", pointer_rot_speed_)){ROS_INFO("error0");}
        while(!nh.getParam("roller/rotation_speed", roller_rot_speed_)){ROS_INFO("error1");}
        while(!nh.getParam("roller/up_down_speed", roller_updown_speed_)){ROS_INFO("error2");}
        while(!nh.getParam("loader/servo_r_angle_push", servo_angles_[Side::RIGHT][ServoState::WIPE])){ROS_INFO("error3");}
        while(!nh.getParam("loader/servo_r_angle_reset", servo_angles_[Side::RIGHT][ServoState::RESET])){ROS_INFO("error4");}
        while(!nh.getParam("loader/servo_l_angle_push", servo_angles_[Side::LEFT][ServoState::WIPE])){ROS_INFO("error5");}
        while(!nh.getParam("loader/servo_l_angle_reset", servo_angles_[Side::LEFT][ServoState::RESET])){ROS_INFO("error6");}
        while(!nh.getParam("loader/motor_angle_load", load_motor_angle_goal_[LoadMotorState::LOAD])){ROS_INFO("error7");}
        while(!nh.getParam("loader/motor_angle_init", load_motor_angle_goal_[LoadMotorState::INIT_POS])){ROS_INFO("error8");}
        while(!nh.getParam("lift/updown_speed", lift_motor_updown_speed_)){ROS_INFO("error9");}
        while(!nh.getParam("lift/motor_bottom_angle", lift_motor_angle_goal_[LiftMotorState::BOTTOM])){ROS_INFO("error10");}
        while(!nh.getParam("lift/motor_middle_angle", lift_motor_angle_goal_[LiftMotorState::MIDDLE])){ROS_INFO("error11");}
        while(!nh.getParam("lift/motor_top_angle", lift_motor_angle_goal_[LiftMotorState::TOP])){ROS_INFO("error12");}

        dura_ = new ros::Duration(sleep_t_);
        dura2_ = new ros::Duration(sleep_t_/2.0);
        dura_short_ = new ros::Duration(0.005);
    }

    ~Shooter2()
    {
        delete dura_;
        delete dura2_;
        delete dura_short_;
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
    ros::init(argc, argv, "shooter2");
    ros::NodeHandle nh;

    Shooter2 shooter(nh);

    shooter.startSpin();

}