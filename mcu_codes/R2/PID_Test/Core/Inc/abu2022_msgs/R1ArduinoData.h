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
      typedef int32_t _lift_motor_angle_type;
      _lift_motor_angle_type lift_motor_angle;
      typedef int32_t _load_motor_angle_type;
      _load_motor_angle_type load_motor_angle;
      typedef bool _lift_motor_sw_type;
      _lift_motor_sw_type lift_motor_sw;

    R1ArduinoData():
      lift_motor_angle(0),
      load_motor_angle(0),
      lift_motor_sw(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_lift_motor_angle;
      u_lift_motor_angle.real = this->lift_motor_angle;
      *(outbuffer + offset + 0) = (u_lift_motor_angle.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_lift_motor_angle.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_lift_motor_angle.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_lift_motor_angle.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->lift_motor_angle);
      union {
        int32_t real;
        uint32_t base;
      } u_load_motor_angle;
      u_load_motor_angle.real = this->load_motor_angle;
      *(outbuffer + offset + 0) = (u_load_motor_angle.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_load_motor_angle.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_load_motor_angle.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_load_motor_angle.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->load_motor_angle);
      union {
        bool real;
        uint8_t base;
      } u_lift_motor_sw;
      u_lift_motor_sw.real = this->lift_motor_sw;
      *(outbuffer + offset + 0) = (u_lift_motor_sw.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->lift_motor_sw);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_lift_motor_angle;
      u_lift_motor_angle.base = 0;
      u_lift_motor_angle.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_lift_motor_angle.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_lift_motor_angle.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_lift_motor_angle.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->lift_motor_angle = u_lift_motor_angle.real;
      offset += sizeof(this->lift_motor_angle);
      union {
        int32_t real;
        uint32_t base;
      } u_load_motor_angle;
      u_load_motor_angle.base = 0;
      u_load_motor_angle.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_load_motor_angle.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_load_motor_angle.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_load_motor_angle.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->load_motor_angle = u_load_motor_angle.real;
      offset += sizeof(this->load_motor_angle);
      union {
        bool real;
        uint8_t base;
      } u_lift_motor_sw;
      u_lift_motor_sw.base = 0;
      u_lift_motor_sw.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->lift_motor_sw = u_lift_motor_sw.real;
      offset += sizeof(this->lift_motor_sw);
     return offset;
    }

    const char * getType(){ return "abu2022_msgs/R1ArduinoData"; };
    const char * getMD5(){ return "8d56dcdd95408dade4b351de93da3cd5"; };

  };

}
#endif
