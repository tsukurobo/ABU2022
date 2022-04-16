#ifndef _ROS_abu2022_msgs_BaseCmd_h
#define _ROS_abu2022_msgs_BaseCmd_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace abu2022_msgs
{

  class BaseCmd : public ros::Msg
  {
    public:
      typedef float _vx_type;
      _vx_type vx;
      typedef float _vy_type;
      _vy_type vy;
      typedef float _omega_type;
      _omega_type omega;

    BaseCmd():
      vx(0),
      vy(0),
      omega(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_vx;
      u_vx.real = this->vx;
      *(outbuffer + offset + 0) = (u_vx.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_vx.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_vx.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_vx.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->vx);
      union {
        float real;
        uint32_t base;
      } u_vy;
      u_vy.real = this->vy;
      *(outbuffer + offset + 0) = (u_vy.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_vy.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_vy.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_vy.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->vy);
      union {
        float real;
        uint32_t base;
      } u_omega;
      u_omega.real = this->omega;
      *(outbuffer + offset + 0) = (u_omega.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_omega.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_omega.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_omega.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->omega);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_vx;
      u_vx.base = 0;
      u_vx.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_vx.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_vx.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_vx.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->vx = u_vx.real;
      offset += sizeof(this->vx);
      union {
        float real;
        uint32_t base;
      } u_vy;
      u_vy.base = 0;
      u_vy.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_vy.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_vy.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_vy.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->vy = u_vy.real;
      offset += sizeof(this->vy);
      union {
        float real;
        uint32_t base;
      } u_omega;
      u_omega.base = 0;
      u_omega.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_omega.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_omega.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_omega.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->omega = u_omega.real;
      offset += sizeof(this->omega);
     return offset;
    }

    const char * getType(){ return "abu2022_msgs/BaseCmd"; };
    const char * getMD5(){ return "fd8c986f429513382e3f32c493ab716c"; };

  };

}
#endif
