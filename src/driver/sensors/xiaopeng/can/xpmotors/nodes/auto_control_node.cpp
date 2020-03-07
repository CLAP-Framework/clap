/**
 * @file auto_control_node.cpp
 * @brief xiaopeng auto control message:
 * 	msg.AutoMode
 * 	msg.VoiceAlarm
 * 	msg.TurnLight
 * 	msg.TarSpeed	request speed of auto
 * 	msg.EPSAngle	request EPS angle control
 *
 * @author yong.zhu@novauto.com.cn
 * @version 0.0.1
 * @date 2019-10-24
 */

#include <iostream>
#include <string>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "param.h"
#include "xpmotors_can_msgs/AutoCtlReq.h"
#include "XUdp.h"
#include "CanFrame.h"
#include "CanCOM_5F0.hpp"

using namespace ros;
using namespace std;

string ipAddr;
unsigned int ipPort;

/*
void BuildCanFrame(int16 steeringAngle, int16 wheelSpeed, uint8 controMode,
		uint8 alarmLevel, uint8 lightLevel)
{
	steeringAngle = steeringAngle * 50;
	wheelSpeed = wheelSpeed * 256;
	uint8 autoM;
	uint8 voiceA;
	uint8 lightL;
*/

/**
 * @brief assemble car control protocol.
 *
 * @param steeringAngleIn 
 * @param wheelspeedIn
 * @param control_mode
 * @param alarm_level
 * @param light_level
 */
void
SendCanFrame (int16 steeringAngleIn, int16 wheelspeedIn, uint8 control_mode,
	      uint8 alarm_level, uint8 light_level)
{

  steeringAngleIn = steeringAngleIn * 50;
  wheelspeedIn = wheelspeedIn * 256;
  uint8 automode1;
  uint8 voicealarm1;
  uint8 turnlight1;
  uint16 epsangele_req1;
  int16 tarspeed_req1;
  float mid_epsangle_req1;

  mid_epsangle_req1 = steeringAngleIn + 30000;
  epsangele_req1 = (uint16) mid_epsangle_req1;
  tarspeed_req1 = (int16) wheelspeedIn;

  uint8 Buf5f0[13];
  XUdp xudp;

  if (control_mode == 0x00 || control_mode == 0x01)
    automode1 = control_mode;
  //   else automode1=automode1;

  if (alarm_level >= 0 && alarm_level <= 3)
    voicealarm1 = alarm_level;
  //ICV_LOG_INFO<<"XIAOPENG alarm1";
  //   else voicealarm1=voicealarm1;
  if (light_level >= 0 && light_level <= 4)
    turnlight1 = light_level;
  //   else turnlight1=turnlight1;
  //if(epsangele_req1>=60000) epsangele_req1=60000;
  //   else if (epsangele_req1<=0) epsangele_req1=0;

  if (epsangele_req1 >= 60000)
    epsangele_req1 = 60000;
  else if (epsangele_req1 <= 0)
    epsangele_req1 = 0;

  if (tarspeed_req1  >= 2048)
    tarspeed_req1 = 2048;
  else if (tarspeed_req1 <= -2048)
    tarspeed_req1 = -2048;

  //=======================0f0============================
  memset (Buf5f0, 0, sizeof (Buf5f0));
  Buf5f0[0] = 0x08;		//0x08
  Buf5f0[1] = 0x00;
  Buf5f0[2] = 0x00;
  Buf5f0[3] = 0x05;
  Buf5f0[4] = 0xf0;

  //////////////data segment/////////////
  Buf5f0[5] = ((voicealarm1 << 4) | (automode1)) & 0xff;	//dns 调试
  Buf5f0[6] = tarspeed_req1 & 0x00ff;
  Buf5f0[7] = (tarspeed_req1 & 0xff00) >> 8;
  Buf5f0[8] = 0x00;
  Buf5f0[9] = 0x00;
  Buf5f0[10] = epsangele_req1;
  Buf5f0[11] = epsangele_req1 >> 8;
  Buf5f0[12] = turnlight1;

  xudp.Send (ipAddr.c_str (), ipPort, Buf5f0, 13);	// Send to Auto.

  // std::cout << "voice and mode " << (int16) Buf5f0[5] << std::endl;
  // std::cout << "speed L " << (int16) Buf5f0[6] << std::endl;
  // std::cout << "speed H " << (int16) Buf5f0[7] << std::endl;
  // std::cout << "eps angle request " << epsangele_req1 << std::endl;
  // std::cout << "angle L " << (int16) Buf5f0[10] << std::endl;
  // std::cout << "angle H " << (int16) Buf5f0[11] << std::endl;
  // std::cout << "-------Send control message ---------" << std::endl;
}

void
callBack (const xpmotors_can_msgs::AutoCtlReq & msg)
{
  // cout << "received messages: " << endl;
  // cout << "auto mode " << (uint8) msg.AutoMode << endl;
  // cout << "VoiceAlarm " << (uint8) msg.VoiceAlarm << endl;
  // cout << "tar speed " << (int16) msg.TarSpeedReq << endl;
  // cout << "EPS angle " << (uint16) msg.EPSAngleReq << endl;
  // cout << "TurnLight " << (uint8) msg.TurnLight << endl;
  // usleep(1);
  SendCanFrame (msg.EPSAngleReq, msg.TarSpeedReq, msg.AutoMode,
	msg.VoiceAlarm, msg.TurnLight);
}

int
main (int argc, char **argv)
{
  init (argc, argv, "auto_control_node");
  NodeHandle n;

  ipAddr = getParam < string > ("auto_control/addr", "192.168.110.101");
  ipPort = getParam < int >("auto_control/port", 4001);

  cout << "ip addr = " << ipAddr << endl;
  cout << "port = " << ipPort << endl;

  Subscriber sub = n.subscribe ("/xp/auto_control", 1000, callBack);
  spin ();
  return 0;
}
