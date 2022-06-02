////ローラー射出機構用プログラム////
#include <ros.h>
#include <abu2022_msgs/R1ArduinoCmd.h>

#include "Cubic.h"
Cubic motor0;
Cubic motor1;//インスタンス作成
//int enc1,enc2;

void ctrlRoller(const abu2022_msgs::R1ArduinoCmd &);

enum Shooter2Actuator
{
  PointerMotor = 0,
  RollerUpDownMotor,
  RollerMotorR,
  RollerMotorL
};

ros::NodeHandle nh;
ros::Subscriber<abu2022_msgs::R1ArduinoCmd> sub("arduino_cmd" , &ctrlRoller);
//abu2022_msgs::R1ArduinoCmd c;

void setup() {
    nh.initNode();
    nh.subscribe(sub);
//  Serial.begin(115200);
  
  motor0.begin(0);//motor1をモーター1として使用
  motor1.begin(1);//motor2をモーター2として使用

  //初期値
  motor0.put(0);
  motor1.put(0);
  Cubic::send();
  
//  c.actuator_id = RollerMotorR;
//  c.value = 50;
}

void loop() {
  nh.spinOnce();
  delay(10);
  //  for (int i = 0; i < 256; i++) {
  //    motor0.put(i);//motor0への値を配列に格納
  //    motor1.put(-i); //motor1への値を配列に格納
  //    motor1.send();//spi通信を行う
  //    delay(10);
  //    //motor1.check();
  //  }
  //  for (int i = 0; i < 256; i++) {
  //    motor0.put(255 - i); //motor0への値を配列に格納
  //    motor1.put(-255 + i); //motor1への値を配列に格納
  //    Cubic::send();//spi通信を行う。
  //    //motor1.check();
  //    delay(10);
  //  }
  //  static int enc0;
  //  motor0 >> enc0;
  /*motor2.putbuf(-120);
    motor3.putbuf(130);
    motor4.putbuf(-140);
    motor5.putbuf(150);
    motor6.putbuf(-160);
    motor7.putbuf(170);

    motor1.check();
    motor1 >> enc1; //enc1にエンコーダの値を格納
    motor2 >> enc2;
    Serial.print(enc1);
    Serial.print("   ");
    Serial.println(enc2);*/
//    ctrlRoller(c);

}

void ctrlRoller(const abu2022_msgs::R1ArduinoCmd &cmd)
{
  if (cmd.actuator_id == RollerMotorR)
  {
    motor0.put(cmd.value);
//    Serial.println(cmd.value);
  }
  else if (cmd.actuator_id == RollerMotorL)
  {
    motor1.put(cmd.value);
  }

  Cubic::send();
}
