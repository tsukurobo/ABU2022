#ifndef _ROS_abu2022_msgs_R2DiskCmd_h
#define _ROS_abu2022_msgs_R2DiskCmd_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace abu2022_msgs
{

  class R2DiskCmd : public ros::Msg
  {
    public:
      typedef int32_t _up_type;
      _up_type up;
      typedef int32_t _down_type;
      _down_type down;
      typedef int32_t _act_type;
      _act_type act;

    R2DiskCmd():
      up(0),
      down(0),
      act(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_up;
      u_up.real = this->up;
      *(outbuffer + offset + 0) = (u_up.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_up.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_up.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_up.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->up);
      union {
        int32_t real;
        uint32_t base;
      } u_down;
      u_down.real = this->down;
      *(outbuffer + offset + 0) = (u_down.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_down.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_down.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_down.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->down);
      union {
        int32_t real;
        uint32_t base;
      } u_act;
      u_act.real = this->act;
      *(outbuffer + offset + 0) = (u_act.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_act.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_act.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_act.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->act);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_up;
      u_up.base = 0;
      u_up.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_up.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_up.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_up.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->up = u_up.real;
      offset += sizeof(this->up);
      union {
        int32_t real;
        uint32_t base;
      } u_down;
      u_down.base = 0;
      u_down.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_down.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_down.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_down.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->down = u_down.real;
      offset += sizeof(this->down);
      union {
        int32_t real;
        uint32_t base;
      } u_act;
      u_act.base = 0;
      u_act.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_act.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_act.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_act.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->act = u_act.real;
      offset += sizeof(this->act);
     return offset;
    }

    virtual const char * getType() override { return "abu2022_msgs/R2DiskCmd"; };
    virtual const char * getMD5() override { return "b447e267365d747d83aab9df46b87b7b"; };

  };

}
#endif
