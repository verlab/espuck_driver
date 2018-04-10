#ifndef _ROS_espuck_driver_Proximity_h
#define _ROS_espuck_driver_Proximity_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace espuck_driver
{

  class Proximity : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t proximity_length;
      typedef int16_t _proximity_type;
      _proximity_type st_proximity;
      _proximity_type * proximity;

    Proximity():
      header(),
      proximity_length(0), proximity(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->proximity_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->proximity_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->proximity_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->proximity_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->proximity_length);
      for( uint32_t i = 0; i < proximity_length; i++){
      union {
        int16_t real;
        uint16_t base;
      } u_proximityi;
      u_proximityi.real = this->proximity[i];
      *(outbuffer + offset + 0) = (u_proximityi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_proximityi.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->proximity[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t proximity_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      proximity_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      proximity_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      proximity_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->proximity_length);
      if(proximity_lengthT > proximity_length)
        this->proximity = (int16_t*)realloc(this->proximity, proximity_lengthT * sizeof(int16_t));
      proximity_length = proximity_lengthT;
      for( uint32_t i = 0; i < proximity_length; i++){
      union {
        int16_t real;
        uint16_t base;
      } u_st_proximity;
      u_st_proximity.base = 0;
      u_st_proximity.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_proximity.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->st_proximity = u_st_proximity.real;
      offset += sizeof(this->st_proximity);
        memcpy( &(this->proximity[i]), &(this->st_proximity), sizeof(int16_t));
      }
     return offset;
    }

    const char * getType(){ return "espuck_driver/Proximity"; };
    const char * getMD5(){ return "c460034d34e6be4d11f97feacc00d97c"; };

  };

}
#endif