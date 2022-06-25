//////ball_handlerとlagoridisk2_handlerをガッチャンコしたプログラム//////

#include <ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include "Actuator.h"

#define PID_PERIOD 10 //period of PID control loop [ms]

void onReceiveCmd(const std_msgs::Int16MultiArray &);
void dataSender(const CustomDataStrageStructure::List<long> &);

ros::NodeHandle nh;
std_msgs::Int32MultiArray ard_cmd;
ros::Publisher data_pub("integrated_data", &ard_cmd);
ros::Subscriber<std_msgs::Int16MultiArray> cmd_sub("integrated_cmd", &onReceiveCmd);

Actuator::AirCylinder *ac_hand, *ac_rot;    //アーム開閉・回転用
Actuator::DCMotorEx *motor_arm, *motor_movable_rail;    //アーム昇降用
Actuator::DCMotorEx *motor_boh;     //BOH回転用
Actuator::AirCylinder *ac_exp, *ac_lift, *ac_catch;     //ボール回収機構用
Actuator::ActuatorContainer container(PID_PERIOD, dataSender);

const float arm_belt_radius = 9.56, //unit: [mm]
            movable_rail_belt_radius = 11.0; //unit: [mm]
//            arm_height_top = 590.0; //unit: [mm]
const int touch_sens_top_pin = 4, touch_sens_bottom_pin = 5;


void setup() 
{               
    //pin assign for lagoridisk_handler
    const uint8_t md_arm_addr = 0x1B, md_mr_addr = 0x15;
    const int ac_hand_pin1 = 2, ac_hand_pin2 = 14,
              ac_rot_pin1 = 9, ac_rot_pin2 = 7;
    const int enc_resol = 4096;
    //the value of CCR increases or decreases by 3
    //every time container.update() is executed
    const float motor_acc = 50.0;

    //pin assign for ball_handler
    const uint8_t md_boh_addr = 0x22;
    const int ac_exp_pin1 = 16, ac_exp_pin2 = 17,
              ac_lift_pin1 = 24, ac_lift_pin2 = 25,
              ac_catch_pin1 = 19, ac_catch_pin2 = 22;
    const float motor_acc2 = 2;

    pinMode(touch_sens_top_pin, INPUT_PULLUP);
    pinMode(touch_sens_bottom_pin, INPUT_PULLUP);

    nh.getHardware()->setBaud(115200);
    nh.initNode();
    nh.advertise(data_pub);
    nh.subscribe(cmd_sub);
    
    ard_cmd.data = (long *)malloc(sizeof(long)*4);
    ard_cmd.data_length = 4;

    motor_arm = new Actuator::DCMotorEx(md_arm_addr, enc_resol, motor_acc);
    motor_movable_rail = new Actuator::DCMotorEx(md_mr_addr, enc_resol, motor_acc);
    ac_hand = new Actuator::AirCylinder(ac_hand_pin1, ac_hand_pin2);
    ac_rot = new Actuator::AirCylinder(ac_rot_pin1, ac_rot_pin2);
    ac_exp = new Actuator::AirCylinder(ac_exp_pin1, ac_exp_pin2);
    ac_lift = new Actuator::AirCylinder(ac_lift_pin1, ac_lift_pin2);
    ac_catch = new Actuator::AirCylinder(ac_catch_pin1, ac_catch_pin2);    
    motor_boh = new Actuator::DCMotorEx(md_boh_addr, 0, motor_acc2);

    //you must register actuators in an appropriate order
    /* the list of actuators
     * 0. motor: arm
     * 1. motor: movable rail
     * 2. air cylinder: catch disk
     * 3. air cylinder: rotate arm
     * 4. air cylinder: expand
     * 5. air cylinder: lift
     * 6. air cylinder: catch ball
     * 7. motor: rotate BOH
    */
    container.append(motor_arm);
    container.append(motor_movable_rail);
    container.append(ac_hand);
    container.append(ac_rot);
    container.append(ac_exp);
    container.append(ac_lift);
    container.append(ac_catch);
    container.append(motor_boh);
    
    //Serial.begin(250000);
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
        container.resetAllActuators(); //set default value to all actuators in the container
}

void onReceiveCmd(const std_msgs::Int16MultiArray &cmd)
{
    container.setValue(cmd.data[0], cmd.data[1]);
}

void dataSender(const CustomDataStrageStructure::List<long> &result_v)
{
    CustomDataStrageStructure::List<long>::Iterator itr = result_v.begin();
    ard_cmd.data[0] = (float)(*itr)*arm_belt_radius*0.01; //angle of motor_arm unit: [mm]
    itr++;
    ard_cmd.data[1] = (float)(*itr)*movable_rail_belt_radius*0.01; //angle of motor_movable_rail unit: [mm]
    ard_cmd.data[2] = !digitalRead(touch_sens_top_pin);
    ard_cmd.data[3] = !digitalRead(touch_sens_bottom_pin);

    data_pub.publish(&ard_cmd);
}
