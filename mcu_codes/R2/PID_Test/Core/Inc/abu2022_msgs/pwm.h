#ifndef _ROS_abu2022_msgs_pwm_h
#define _ROS_abu2022_msgs_pwm_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace abu2022_msgs
{

  class pwm : public ros::Msg
  {
    public:
      typedef int8_t _on_off_type;
      _on_off_type on_off;
      typedef int8_t _freq_type;
      _freq_type freq;
      typedef int8_t _duty_type;
      _duty_type duty;

    pwm():
      on_off(0),
      freq(0),
      duty(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_on_off;
      u_on_off.real = this->on_off;
      *(outbuffer + offset + 0) = (u_on_off.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->on_off);
      union {
        int8_t real;
        uint8_t base;
      } u_freq;
      u_freq.real = this->freq;
      *(outbuffer + offset + 0) = (u_freq.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->freq);
      union {
        int8_t real;
        uint8_t base;
      } u_duty;
      u_duty.real = this->duty;
      *(outbuffer + offset + 0) = (u_duty.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->duty);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_on_off;
      u_on_off.base = 0;
      u_on_off.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->on_off = u_on_off.real;
      offset += sizeof(this->on_off);
      union {
        int8_t real;
        uint8_t base;
      } u_freq;
      u_freq.base = 0;
      u_freq.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->freq = u_freq.real;
      offset += sizeof(this->freq);
      union {
        int8_t real;
        uint8_t base;
      } u_duty;
      u_duty.base = 0;
      u_duty.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->duty = u_duty.real;
      offset += sizeof(this->duty);
     return offset;
    }

    const char * getType(){ return "abu2022_msgs/pwm"; };
    const char * getMD5(){ return "20babff89d56b350c03c58fd594a9cef"; };

  };

}
#endif
