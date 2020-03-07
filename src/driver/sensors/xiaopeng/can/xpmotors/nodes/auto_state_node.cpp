/**
 * @file auto_state_node.cpp
 * @brief publish msgs:
 * 	Auto state, EPS status, ESC status..
 *
 * @author yong.zhu@novauto.com.cn
 * @version 0.0.1
 * @date 2019-10-24
 */

#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "xpmotors_can_msgs/AutoStateEx.h"
#include "xpmotors_can_msgs/AutoState.h"
#include "xpmotors_can_msgs/ESCStatus.h"
#include "xpmotors_can_msgs/EPSStatus.h"
#include "XUdp.h"
#include "param.h"

// CanNet
#include "structureCanXP.h"
#include "CanFrame.h"
#include "CanEPS_511.hpp"
#include "CanSPD_510.hpp"
#include "CanBT_509.hpp"
#include "CanMotor_50B.hpp"

using namespace ros;
using namespace std;

string ipAddr;
unsigned int ipPort;

int
main (int argc, char **argv)
{
  init (argc, argv, "auto_state_node");
  NodeHandle n;

  XUdp xudp;
  uint8 buf[512] = { 0 };
  char ip[100] = { 0 };
  int len;

  // Received from cannet
  CanSPD_510 xpEsc;
  CanBT_509 xpStateEx;
  CanEPS_511 xpEps;
  CanMotor_50B xpState;

  ipAddr = getParam < string > ("auto_control/addr", "192.168.110.101");
  ipPort = getParam < int >("auto_control/port", 4001);

  Publisher pubStateEx = n.advertise < xpmotors_can_msgs::AutoStateEx > ("/xp/auto_state_ex", 1000);
  Publisher pubState = n.advertise < xpmotors_can_msgs::AutoState > ("/xp/auto_state", 1000);
  Publisher pubEsc = n.advertise < xpmotors_can_msgs::ESCStatus > ("/xp/esc_status", 1000);
  Publisher pubEps = n.advertise < xpmotors_can_msgs::EPSStatus > ("/xp/eps_status", 1000);

  if (xudp.Bind (ipPort) < 0)
    return -1;

  Rate loop_rate (2);
  while (ok ())
    {
      uint8 *p;
      memset (buf, 0, sizeof (buf));
      memset (ip, 0, sizeof (ip));

      len = xudp.Receive (buf, 512, ip);
    //   cout << buf << endl;
    //   cout << " received length: " << len << endl;

      int n = len / 13;
      p = buf;

      if (len % 13 == 0)	// 13 times
	{
	  while (n)
	    {
	      uint16 i, j;

	      int flag = 0;
	      TimestampedCanFrame Tframe;

	      Tframe.frame.id = ((uint16) (*(p+3) << 8) + *(p+4));
	      for (j = 0; j < 8; j++) {
		Tframe.frame.data[j] = *(p+j+5);
	      }

	      switch (Tframe.frame.id)
		{
		case 0x510:
		{

  			xpmotors_can_msgs::ESCStatus escStatus;
			xpEsc.SetData(Tframe);
			xpEsc.decode();
			CanFrameSPD_510 data510 = *(xpEsc.data());

			escStatus.RRWheelSpd = data510.RR_WhlSpd;
			escStatus.LFWheelSpd = data510.LF_WhlSpd;
			escStatus.LRWheelSpd = data510.LR_WhlSpd;
			escStatus.RFWheelSpd = data510.RF_WhlSpd;

			pubEsc.publish(escStatus);
			break;
		}

		case 0x511:
		{
			xpmotors_can_msgs::EPSStatus epsStatus;
			
			xpEps.SetData(Tframe);
			xpEps.decode();
			CanFrameEPS_511 data511 = *(xpEps.data());

			epsStatus.Angle = data511.EPS_angle_ccp;
			epsStatus.AngleSpd = data511.EPS_angle_spd_ccp;
			epsStatus.StrngWhlTorq = data511.EPS_StrngWhlTorq;

			pubEps.publish(epsStatus);
			break;
		}

		case 0x509:
		{

  			xpmotors_can_msgs::AutoStateEx aStateEx;
			xpStateEx.SetData(Tframe);
			xpStateEx.decode();
			CanFrameBT_509 data509 = *(xpStateEx.data());

			aStateEx.StateTurningLight = data509.State_TurningLight_CCP;
			aStateEx.CurDriveMode = data509.CurDriveMode_CCP;
			aStateEx.StateBraking = data509.State_Braking_CCP;

			pubStateEx.publish(aStateEx);
			break;
		}
		case 0x50B:
		{

			xpmotors_can_msgs::AutoState aState;
			xpState.SetData(Tframe);
			xpState.decode();
			CanFrameMotor_50B data50B = *(xpState.data());

			aState.EPBState = data50B.EpbState_CCP;
			aState.GearState = data50B.GearState_CCP;
			aState.BrkPedal = data50B.BrkPedal_CCP;
			aState.AccPedal = data50B.AccPedal_CCP;

			pubState.publish(aState);
			break;
		}
		default:
			break;
			// cout << "other ids." << endl;

		}
	      n--;
	      p = p + 13;
	    }
	}
    }
  return 0;
}

