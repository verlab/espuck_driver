#ifndef _ROS_espuck_driver_Led_h
#define _ROS_espuck_driver_Led_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace espuck_driver
{

  class Led : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef int8_t _index_type;
      _index_type index;
      typedef int8_t _state_type;
      _state_type state;

    Led():
      header(),
      index(0),
      state(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        int8_t real;
        uint8_t base;
      } u_index;
      u_index.real = this->index;
      *(outbuffer + offset + 0) = (u_index.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->index);
      union {
        int8_t real;
        uint8_t base;
      } u_state;
      u_state.real = this->state;
      *(outbuffer + offset + 0) = (u_state.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->state);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        int8_t real;
        uint8_t base;
      } u_index;
      u_index.base = 0;
      u_index.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->index = u_index.real;
      offset += sizeof(this->index);
      union {
        int8_t real;
        uint8_t base;
      } u_state;
      u_state.base = 0;
      u_state.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->state = u_state.real;
      offset += sizeof(this->state);
     return offset;
    }

    const char * getType(){ return "espuck_driver/Led"; };
    const char * getMD5(){ return "f93b5e061d05aacfd44c522df6ab05a4"; };

  };

}
#endif