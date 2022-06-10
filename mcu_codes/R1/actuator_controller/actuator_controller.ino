#include <ros.h>
#include <abu2022_msgs/R1ArduinoCmd.h>
#include <abu2022_msgs/R1ArduinoData.h>
#include "Actuator.h"

#define TOUCH_SENS 8 //タッチセンサが接続されたピン
#define PID_PERIOD 10 //PID制御周期。単位はms

void onReceiveCmd(const abu2022_msgs::R1ArduinoCmd &);
void dataSender(const CustomDataStrageStructure::List<long> &);

ros::NodeHandle nh;
abu2022_msgs::R1ArduinoData ard_data;
ros::Publisher data_pub("arduino_data", &ard_data);
ros::Subscriber<abu2022_msgs::R1ArduinoCmd> cmd_sub("arduino_cmd", &onReceiveCmd);

Actuator::DCMotor *motor_pointer, *motor_roller_updown, *motor_lift_no_pid;
/*Actuator::DCMotorWithVelPID*/
Actuator::Actuator *motor_roller_roller_r, *motor_roller_roller_l;  //ダミー
Actuator::DCMotorWithPosPID *motor_load, *motor_lift;
//Actuator::DCMotor *motor_roller_roller1, *motor_roller_roller2; //一時しのぎ用
Actuator::ServoMotor *servo_r, *servo_l;
Actuator::AirCylinder *ac;
Actuator::ActuatorContainer container(PID_PERIOD, dataSender);

void setup() 
{   
    //速度制御PID用パラメータ
    //const float kp = 6.59, ki = 17.1, kd = 0.15,
    const float kp = 0.115, ki = 0.298, kd = 0.00262,
                vmax = 255.0, tdel = 0.0022, ts = (float)PID_PERIOD/1000.0, k_enc = 0.4;
    const int enc_resol = 1024;
    const int enc_resol2 = 1024;

    //位置制御用PIDパラメータ
    const float kp_p = 8.0, ki_p = 0.0, kd_p = 0.04,
                vmax_p = 150, tdel_p = 0.00125;
                
    //ピンアサイン
    const uint8_t md_pointer_addr = 0x14, md_roller_updown_addr = 0x15, /*md_roller_addrs[2] = {0x77, 0x1B},*/
                  md_load_addr = 0x17, md_lift_addr = 0x25;
    const int servo_r_pin = 2, servo_l_pin = 5, ac_pin1 = A13, ac_pin2 = A10;

    //台形制御用定数（モーターの加速度 [deg/s^2]）
//    const float rot_acc = 5730.0;//100;
    
    //初期指令
    int servo_r_init_cmd = 0, servo_l_init_cmd = 0;

//    PIDController::PIDSettings pids;
//    pids.kp = kp;
//    pids.ki = ki;
//    pids.kd = kd;
//    pids.ts = ts;
//    pids.tdel = tdel;
//    pids.vmax = vmax;
//    pids.vmin = -vmax;

    PIDController::PIDSettings pids_p;
    pids_p.kp = kp_p;
    pids_p.ki = ki_p;
    pids_p.kd = kd_p;
    pids_p.ts = ts;
    pids_p.tdel = tdel_p;
    pids_p.vmax = vmax_p;
    pids_p.vmin = -vmax_p;

    PIDController::PIDSettings pids_p2;
    pids_p2.kp = kp_p;
    pids_p2.ki = ki_p;
    pids_p2.kd = kd_p;
    pids_p2.ts = ts;
    pids_p2.tdel = tdel_p;
    pids_p2.vmax = 250;
    pids_p2.vmin = -250;

    nh.getHardware()->setBaud(250000);
    nh.initNode();
    nh.advertise(data_pub);
    nh.subscribe(cmd_sub);

    //第3引数はちゃんと指定すること
    while(!nh.getParam("/loader/servo_r_angle_reset", &servo_r_init_cmd, 1)){nh.spinOnce();}
    while(!nh.getParam("/loader/servo_l_angle_reset", &servo_l_init_cmd, 1)){nh.spinOnce();}
    
    motor_pointer = new Actuator::DCMotor(md_pointer_addr);
    motor_roller_updown = new Actuator::DCMotor(md_roller_updown_addr);
//    motor_roller_roller_r = new Actuator::DCMotorWithVelPID(md_roller_addrs[0], pids, 
//                            enc_resol, rot_acc, k_enc);
//    motor_roller_roller_l = new Actuator::DCMotorWithVelPID(md_roller_addrs[1], pids,
//                            enc_resol, rot_acc, k_enc);
    motor_roller_roller_r = new Actuator::Actuator(false);
    motor_roller_roller_l = new Actuator::Actuator(false);
    motor_load = new Actuator::DCMotorWithPosPID(md_load_addr, pids_p, enc_resol2);
    motor_lift = new Actuator::DCMotorWithPosPID(md_lift_addr, pids_p2, enc_resol);
    motor_lift_no_pid = new Actuator::DCMotor(md_lift_addr);
    //motor_roller_roller1 = new Actuator::DCMotor(md_roller_addrs[0]);
    //motor_roller_roller2 = new Actuator::DCMotor(md_roller_addrs[1]);
    servo_r = new Actuator::ServoMotor(servo_r_pin, servo_r_init_cmd);
    servo_l = new Actuator::ServoMotor(servo_l_pin, servo_l_init_cmd);
    ac = new Actuator::AirCylinder(ac_pin1, ac_pin2);

    //appendの順番は大事
    container.append(motor_pointer);
    container.append(motor_roller_updown);
    container.append(motor_roller_roller_r);
    container.append(motor_roller_roller_l);
    container.append(motor_lift);
    container.append(motor_lift_no_pid);
    container.append(servo_r);
    container.append(servo_l);
    container.append(motor_load);
    container.append(ac);

    //Serial.begin(250000);

    //このプログラムにおいては、このwhileループを入れないと上手く動作しないようだ（原因は不明）
    //たぶん、nh.connected()がtrueだろうが、falseだろうが、nh.spinOnce()は実行される必要があるのだと思う
//    while(!nh.connected())
//    {
//        nh.spinOnce();
//        delay(200);
//    }

    pinMode(TOUCH_SENS, INPUT_PULLUP);
}

void loop() 
{
    nh.spinOnce();
    if(nh.connected())
    {
        container.update();
        //nh.spinOnce();
    }
    else
        container.resetAllActuators();     //全ての登録されたアクチュエータにデフォルト値を設定する
        
}

void onReceiveCmd(const abu2022_msgs::R1ArduinoCmd &cmd)
{
    if(cmd.actuator_id == 4)
        motor_lift->setEnablePID(true);
    else if(cmd.actuator_id == 5)
        motor_lift->setEnablePID(false);
    container.setValue(cmd.actuator_id, cmd.value);
}

void dataSender(const CustomDataStrageStructure::List<long> &result_v)
{
    //*(itr++)はエラーとなる。多分評価順序とかの問題かと。
    CustomDataStrageStructure::List<long>::Iterator itr = result_v.begin();
    
//    for(int i=0; i<2; i++) itr++;
    ard_data.lift_motor_angle = *itr;    //lift motorの角度データをard_dataに渡す
    
    itr++;
    ard_data.load_motor_angle = *itr;    //load motorの角度データをard_dataに渡す
    ard_data.lift_motor_sw = !digitalRead(TOUCH_SENS);
//    ard_data.lift_motor_angle = *itr;
//    itr++;
//    ard_data.load_motor_angle = *itr;
    data_pub.publish(&ard_data);    //publish!
}
