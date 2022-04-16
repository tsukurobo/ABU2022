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
      typedef float _delta_x_type;
      _delta_x_type delta_x;
      typedef float _delta_y_type;
      _delta_y_type delta_y;
      typedef float _delta_theta_type;
      _delta_theta_type delta_theta;

    BaseData():
      delta_x(0),
      delta_y(0),
      delta_theta(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_delta_x;
      u_delta_x.real = this->delta_x;
      *(outbuffer + offset + 0) = (u_delta_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_delta_x.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_delta_x.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_delta_x.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->delta_x);
      union {
        float real;
        uint32_t base;
      } u_delta_y;
      u_delta_y.real = this->delta_y;
      *(outbuffer + offset + 0) = (u_delta_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_delta_y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_delta_y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_delta_y.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->delta_y);
      union {
        float real;
        uint32_t base;
      } u_delta_theta;
      u_delta_theta.real = this->delta_theta;
      *(outbuffer + offset + 0) = (u_delta_theta.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_delta_theta.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_delta_theta.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_delta_theta.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->delta_theta);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_delta_x;
      u_delta_x.base = 0;
      u_delta_x.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_delta_x.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_delta_x.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_delta_x.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->delta_x = u_delta_x.real;
      offset += sizeof(this->delta_x);
      union {
        float real;
        uint32_t base;
      } u_delta_y;
      u_delta_y.base = 0;
      u_delta_y.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_delta_y.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_delta_y.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_delta_y.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->delta_y = u_delta_y.real;
      offset += sizeof(this->delta_y);
      union {
        float real;
        uint32_t base;
      } u_delta_theta;
      u_delta_theta.base = 0;
      u_delta_theta.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_delta_theta.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_delta_theta.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_delta_theta.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->delta_theta = u_delta_theta.real;
      offset += sizeof(this->delta_theta);
     return offset;
    }

    const char * getType(){ return "abu2022_msgs/BaseData"; };
    //const char * getMD5(){ return "fc04043e1a1c0ab3a48c0427399533b0"; };
    const char * getMD5(){ return "21510639c45ace458949ef0bb5dfdbf5"; };

  };

}
#endif
