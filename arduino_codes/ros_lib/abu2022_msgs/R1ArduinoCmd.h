#ifndef _ROS_abu2022_msgs_R1ArduinoCmd_h
#define _ROS_abu2022_msgs_R1ArduinoCmd_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace abu2022_msgs
{

  class R1ArduinoCmd : public ros::Msg
  {
    public:
      typedef int8_t _actuator_id_type;
      _actuator_id_type actuator_id;
      typedef int32_t _value_type;
      _value_type value;

    R1ArduinoCmd():
      actuator_id(0),
      value(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_actuator_id;
      u_actuator_id.real = this->actuator_id;
      *(outbuffer + offset + 0) = (u_actuator_id.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->actuator_id);
      union {
        int32_t real;
        uint32_t base;
      } u_value;
      u_value.real = this->value;
      *(outbuffer + offset + 0) = (u_value.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_value.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_value.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_value.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->value);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_actuator_id;
      u_actuator_id.base = 0;
      u_actuator_id.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->actuator_id = u_actuator_id.real;
      offset += sizeof(this->actuator_id);
      union {
        int32_t real;
        uint32_t base;
      } u_value;
      u_value.base = 0;
      u_value.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_value.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_value.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_value.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->value = u_value.real;
      offset += sizeof(this->value);
     return offset;
    }

    virtual const char * getType() override { return "abu2022_msgs/R1ArduinoCmd"; };
    virtual const char * getMD5() override { return "3b6b5c30f17f146a6ec965b03c9821ea"; };

  };

}
#endif
