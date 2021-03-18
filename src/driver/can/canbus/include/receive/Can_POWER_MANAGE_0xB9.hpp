/******************************************************************************
 * @file Can_POWER_MANAGE_0xB9.hpp
 * @brief CAN  receive - POWER_MANAGE_MODULE
 *
 * @author yafei.sun@novauto.com.cn
 * @version 0.0.1
 * @date 2020-10-20
 *****************************************************************************/

#ifndef __Can_POWER_MANAGE_0xB9_H__
#define __Can_POWER_MANAGE_0xB9_H__

#include "structureCanXP.h"

class  Can_POWER_MANAGE_0xB9
{
public:

  Can_POWER_MANAGE_0xB9(){};
  ~Can_POWER_MANAGE_0xB9(){}; 
  void decode(){
  data_.AutoModeKeepEnb = 0;

  uint8 *pData = d_.frame.data;
  int16 temp,temp1,temp2,temp3;

  temp =0;
  temp=(short)(pData[0] & 0x01);
  data_.AutoModeKeepEnb=temp;  
  };    //end of decode()
  CanFrame_POWER_MANAGE_0xB9* data() {return &data_; }; 
  int size() { return sizeof(CanFrame_POWER_MANAGE_0xB9); };
  void SetData(TimestampedCanFrame rec_frame){d_=rec_frame;};
protected:

private:
  CanFrame_POWER_MANAGE_0xB9 data_;
  TimestampedCanFrame d_;
};    //end of class  Can_POWER_MANAGE_0xB9

#endif    //end of __Can_POWER_MANAGE_0xB9_H__
