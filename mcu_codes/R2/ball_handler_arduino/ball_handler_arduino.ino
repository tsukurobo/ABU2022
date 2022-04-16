#include <ros.h>
#include <std_msgs/Int16MultiArray.h>
#include "Actuator.h"

#define PID_PERIOD 10 //period of PID control loop [ms]

void onReceiveCmd(const std_msgs::Int16MultiArray &);
void dataSender(const CustomDataStrageStructure::List<long> &);

ros::NodeHandle nh;
//std_msgs::Int16MultiArray ard_cmd;
//ros::Publisher data_pub("arduino_data", &ard_data);
ros::Subscriber<std_msgs::Int16MultiArray> cmd_sub("ball_handler_cmd", &onReceiveCmd);

Actuator::DCMotor *motor_boh;
Actuator::AirCylinder *ac_exp, *ac_lift, *ac_catch;
Actuator::ActuatorContainer container(PID_PERIOD, dataSender);

void setup() 
{               
    //pin assign
    const uint8_t md_boh_addr = 0x22;
    const int ac_exp_pin1 = A4, ac_exp_pin2 = A5, ac_lift_pin1 = A0, ac_lift_pin2 = A1,
                ac_catch_pin1 = A2, ac_catch_pin2 = A3;

    nh.getHardware()->setBaud(250000);
    nh.initNode();
    //nh.advertise(data_pub);
    nh.subscribe(cmd_sub);
    
    motor_boh = new Actuator::DCMotor(md_boh_addr);
    ac_exp = new Actuator::AirCylinder(ac_exp_pin1, ac_exp_pin2);
    ac_lift = new Actuator::AirCylinder(ac_lift_pin1, ac_lift_pin2);
    ac_catch = new Actuator::AirCylinder(ac_catch_pin1, ac_catch_pin2);

    //you must register actuators in an appropriate order
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
        container.resetAllActuators();     //set default value to all actuators in the container
        
}

void onReceiveCmd(const std_msgs::Int16MultiArray &cmd)
{
    container.setValue(cmd.data[0], cmd.data[1]);
}

//this function is not used in this code
void dataSender(const CustomDataStrageStructure::List<long> &result_v)
{
}
