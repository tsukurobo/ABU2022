#ifndef _ROS_abu2022_msgs_BaseData_h
#define _ROS_abu2022_msgs_BaseData_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace abu2022_msgs
{

  class BaseData : public ros::Msg
  {
    public:
      typedef float _x_type;
      _x_type x;
      typedef float _y_type;
      _y_type y;
      typedef float _theta_type;
      _theta_type theta;
      typedef float _e1_type;
      _e1_type e1;
      typedef float _e2_type;
      _e2_type e2;
      typedef float _e3_type;
      _e3_type e3;
      typedef float _e4_type;
      _e4_type e4;

    BaseData():
      x(0),
      y(0),
      theta(0),
      e1(0),
      e2(0),
      e3(0),
      e4(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_x;
      u_x.real = this->x;
      *(outbuffer + offset + 0) = (u_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_x.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_x.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_x.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->x);
      union {
        float real;
        uint32_t base;
      } u_y;
      u_y.real = this->y;
      *(outbuffer + offset + 0) = (u_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_y.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->y);
      union {
        float real;
        uint32_t base;
      } u_theta;
      u_theta.real = this->theta;
      *(outbuffer + offset + 0) = (u_theta.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_theta.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_theta.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_theta.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->theta);
      union {
        float real;
        uint32_t base;
      } u_e1;
      u_e1.real = this->e1;
      *(outbuffer + offset + 0) = (u_e1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_e1.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_e1.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_e1.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->e1);
      union {
        float real;
        uint32_t base;
      } u_e2;
      u_e2.real = this->e2;
      *(outbuffer + offset + 0) = (u_e2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_e2.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_e2.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_e2.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->e2);
      union {
        float real;
        uint32_t base;
      } u_e3;
      u_e3.real = this->e3;
      *(outbuffer + offset + 0) = (u_e3.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_e3.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_e3.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_e3.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->e3);
      union {
        float real;
        uint32_t base;
      } u_e4;
      u_e4.real = this->e4;
      *(outbuffer + offset + 0) = (u_e4.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_e4.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_e4.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_e4.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->e4);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_x;
      u_x.base = 0;
      u_x.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_x.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_x.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_x.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->x = u_x.real;
      offset += sizeof(this->x);
      union {
        float real;
        uint32_t base;
      } u_y;
      u_y.base = 0;
      u_y.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_y.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_y.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_y.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->y = u_y.real;
      offset += sizeof(this->y);
      union {
        float real;
        uint32_t base;
      } u_theta;
      u_theta.base = 0;
      u_theta.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_theta.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_theta.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_theta.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->theta = u_theta.real;
      offset += sizeof(this->theta);
      union {
        float real;
        uint32_t base;
      } u_e1;
      u_e1.base = 0;
      u_e1.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_e1.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_e1.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_e1.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->e1 = u_e1.real;
      offset += sizeof(this->e1);
      union {
        float real;
        uint32_t base;
      } u_e2;
      u_e2.base = 0;
      u_e2.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_e2.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_e2.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_e2.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->e2 = u_e2.real;
      offset += sizeof(this->e2);
      union {
        float real;
        uint32_t base;
      } u_e3;
      u_e3.base = 0;
      u_e3.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_e3.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_e3.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_e3.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->e3 = u_e3.real;
      offset += sizeof(this->e3);
      union {
        float real;
        uint32_t base;
      } u_e4;
      u_e4.base = 0;
      u_e4.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_e4.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_e4.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_e4.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->e4 = u_e4.real;
      offset += sizeof(this->e4);
     return offset;
    }

    virtual const char * getType() override { return "abu2022_msgs/BaseData"; };
    virtual const char * getMD5() override { return "fc04043e1a1c0ab3a48c0427399533b0"; };

  };

}
#endif
