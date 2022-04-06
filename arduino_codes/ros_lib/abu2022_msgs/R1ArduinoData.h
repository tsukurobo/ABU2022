#ifndef _ROS_abu2022_msgs_R1ArduinoData_h
#define _ROS_abu2022_msgs_R1ArduinoData_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace abu2022_msgs
{

  class R1ArduinoData : public ros::Msg
  {
    public:
      typedef int32_t _rot_vel_motor_roller1_type;
      _rot_vel_motor_roller1_type rot_vel_motor_roller1;
      typedef int32_t _rot_vel_motor_roller2_type;
      _rot_vel_motor_roller2_type rot_vel_motor_roller2;

    R1ArduinoData():
      rot_vel_motor_roller1(0),
      rot_vel_motor_roller2(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_rot_vel_motor_roller1;
      u_rot_vel_motor_roller1.real = this->rot_vel_motor_roller1;
      *(outbuffer + offset + 0) = (u_rot_vel_motor_roller1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rot_vel_motor_roller1.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_rot_vel_motor_roller1.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_rot_vel_motor_roller1.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->rot_vel_motor_roller1);
      union {
        int32_t real;
        uint32_t base;
      } u_rot_vel_motor_roller2;
      u_rot_vel_motor_roller2.real = this->rot_vel_motor_roller2;
      *(outbuffer + offset + 0) = (u_rot_vel_motor_roller2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rot_vel_motor_roller2.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_rot_vel_motor_roller2.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_rot_vel_motor_roller2.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->rot_vel_motor_roller2);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_rot_vel_motor_roller1;
      u_rot_vel_motor_roller1.base = 0;
      u_rot_vel_motor_roller1.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rot_vel_motor_roller1.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_rot_vel_motor_roller1.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_rot_vel_motor_roller1.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->rot_vel_motor_roller1 = u_rot_vel_motor_roller1.real;
      offset += sizeof(this->rot_vel_motor_roller1);
      union {
        int32_t real;
        uint32_t base;
      } u_rot_vel_motor_roller2;
      u_rot_vel_motor_roller2.base = 0;
      u_rot_vel_motor_roller2.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rot_vel_motor_roller2.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_rot_vel_motor_roller2.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_rot_vel_motor_roller2.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->rot_vel_motor_roller2 = u_rot_vel_motor_roller2.real;
      offset += sizeof(this->rot_vel_motor_roller2);
     return offset;
    }

    virtual const char * getType() override { return "abu2022_msgs/R1ArduinoData"; };
    virtual const char * getMD5() override { return "3b2fa374f3318552b1f3575a64ef6392"; };

  };

}
#endif
