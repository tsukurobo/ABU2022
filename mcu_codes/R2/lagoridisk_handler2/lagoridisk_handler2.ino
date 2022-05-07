#include <ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include "Actuator.h"

#define PID_PERIOD 10 //period of PID control loop [ms]

void onReceiveCmd(const std_msgs::Int16MultiArray &);
void dataSender(const CustomDataStrageStructure::List<long> &);

ros::NodeHandle nh;
std_msgs::Int32MultiArray ard_cmd;
ros::Publisher data_pub("liftdisk2_data", &ard_cmd);
ros::Subscriber<std_msgs::Int16MultiArray> cmd_sub("liftdisk2_cmd", &onReceiveCmd);

Actuator::AirCylinder *ac_hand, *ac_rot;
Actuator::DCMotorEx *motor_arm, *motor_movable_rail;
Actuator::ActuatorContainer container(PID_PERIOD, dataSender);

const float arm_belt_radius = 9.56, //unit: [mm]
            movable_rail_belt_radius = 11.0; //unit: [mm]
//            arm_height_top = 590.0; //unit: [mm]
const int touch_sens_top_pin = 5, touch_sens_bottom_pin = 3;

void setup() 
{               
    //pin assign
    const uint8_t md_arm_addr = 0x15, md_mr_addr = 0x17;
    const int ac_hand_pin1 = 11, ac_hand_pin2 = 9,
              ac_rot_pin1 = 10, ac_rot_pin2 = 7;
    const int enc_resol = 4096;
    //the value of CCR increases or decreases by 3
    //every time container.update() is executed
    const float motor_acc = 50.0;

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

    //you must register actuators in an appropriate order
    container.append(motor_arm);
    container.append(motor_movable_rail);
    container.append(ac_hand);
    container.append(ac_rot);

    //Serial.begin(250000);
    pinMode(touch_sens_top_pin, INPUT_PULLUP);
    pinMode(touch_sens_bottom_pin, INPUT_PULLUP);
    
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
    ard_cmd.data[0] = (float)(*itr)*arm_belt_radius*0.01; //unit: [mm]
    itr++;
    ard_cmd.data[1] = (float)(*itr)*movable_rail_belt_radius*0.01; //unit: [mm]
    ard_cmd.data[2] = !digitalRead(touch_sens_top_pin);
    ard_cmd.data[3] = !digitalRead(touch_sens_bottom_pin);

    data_pub.publish(&ard_cmd);
}
