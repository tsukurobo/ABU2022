#include <ros.h>
#include <abu2022_msgs/R1ArduinoCmd.h>
#include <abu2022_msgs/R1ArduinoData.h>
#include "Actuator.h"

#define PID_PERIOD 10 //PID制御周期。単位はms

void onReceiveCmd(const abu2022_msgs::R1ArduinoCmd &);
void dataSender(const CustomDataStrageStructure::List<long> &);

ros::NodeHandle nh;
abu2022_msgs::R1ArduinoData ard_data;
ros::Publisher data_pub("arduino_data", &ard_data);
ros::Subscriber<abu2022_msgs::R1ArduinoCmd> cmd_sub("arduino_cmd", &onReceiveCmd);

Actuator::DCMotor *motor_pointer, *motor_roller_updown;
Actuator::DCMotorWithVelPID *motor_roller_roller1, *motor_roller_roller2;
//Actuator::DCMotor *motor_roller_roller1, *motor_roller_roller2; //一時しのぎ用
Actuator::ServoMotor *servo_lever1, *servo_lever2, *servo_lever3;
Actuator::ActuatorContainer container(PID_PERIOD, dataSender);

void setup() 
{   
    //To do: 後でこいつら調整する
    //PID制御用パラメータ
    const float kp = 6.59, ki = 17.1, kd = 0.15,
                vmax = 255.0, tdel = 0.0022, ts = (float)PID_PERIOD/1000.0, k_enc = 0.4;
    const int enc_resol = 4096;
    
    //ピンアサイン
    const uint8_t md_pointer_addr = 0x14, md_roller_updown_addr = 0x15, md_roller_addrs[2] = {0x77, 0x25};
    const int servo_pin1 = 5, servo_pin2 = 2, servo_pin3 = 8;

    //台形制御用定数（モーターの加速度）
    const float rot_acc = 100.0;
    
    //初期指令
    const int servo_lever1_init_cmd = 0, servo_lever2_init_cmd = 0, servo_lever3_init_cmd = 0;

    PIDController::PIDSettings pids;
    pids.kp = kp;
    pids.ki = ki;
    pids.kd = kd;
    pids.ts = ts;
    pids.tdel = tdel;
    pids.vmax = vmax;
    pids.vmin = -vmax;
    
    motor_pointer = new Actuator::DCMotor(md_pointer_addr);
    motor_roller_updown = new Actuator::DCMotor(md_roller_updown_addr);
    //一時しのぎ仕様
    motor_roller_roller1 = new Actuator::DCMotorWithVelPID(md_roller_addrs[0], pids, 
                            enc_resol, rot_acc, k_enc);
    motor_roller_roller2 = new Actuator::DCMotorWithVelPID(md_roller_addrs[1], pids,
                            enc_resol, rot_acc, k_enc);
    //motor_roller_roller1 = new Actuator::DCMotor(md_roller_addrs[0]);
    //motor_roller_roller2 = new Actuator::DCMotor(md_roller_addrs[1]);
    servo_lever1 = new Actuator::ServoMotor(servo_pin1, servo_lever1_init_cmd);
    servo_lever2 = new Actuator::ServoMotor(servo_pin2, servo_lever2_init_cmd);
    servo_lever3 = new Actuator::ServoMotor(servo_pin3, servo_lever3_init_cmd);

    //appendの順番は大事
    container.append(motor_pointer);
    container.append(motor_roller_updown);
    container.append(motor_roller_roller1);
    container.append(motor_roller_roller2);
    container.append(servo_lever1);
    container.append(servo_lever2);
    container.append(servo_lever3);

    nh.getHardware()->setBaud(250000);
    nh.initNode();
    nh.advertise(data_pub);
    nh.subscribe(cmd_sub);
    //Serial.begin(250000);

    //このプログラムにおいては、このwhileループを入れないと上手く動作しないようだ（原因は不明）
    while(!nh.connected())
    {
        nh.spinOnce();
        delay(200);
    }
}

void loop() 
{
    if(nh.connected())
    {
        nh.spinOnce();
        container.update();
    }
    else
    {
        container.resetAllActuators();     //全ての登録させたアクチュエータにデフォルト値を設定する
    }
}

void onReceiveCmd(const abu2022_msgs::R1ArduinoCmd &cmd)
{
    container.setValue(cmd.actuator_id, cmd.value);
}

void dataSender(const CustomDataStrageStructure::List<long> &result_v)
{
    CustomDataStrageStructure::List<long>::Iterator itr = result_v.begin();
    ard_data.rot_vel_motor_roller1 = *itr;    //Listのデータをard_dataに移す
    itr++;
    ard_data.rot_vel_motor_roller2 = *itr; //*(itr++)はエラーとなる。多分評価順序とかの問題かと。
    data_pub.publish(&ard_data);    //publish!
}
