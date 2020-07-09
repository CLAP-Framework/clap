/**
 * @file us_radar_node.cpp
 * @brief ultra sonic radar info node.
 *
 * @author yong.zhu@novauto.com.cn
 * @version 0.0.1
 * @date 2019-10-24
 * @author yafei.sun@novauto.com.cn
 * @version 0.0.2
 * @date 2020-01-31
 */

#include <iostream>
#include <string>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "param.h"
#include "XUdp.h"
#include "us_radar_msgs/DetectionsArray.h"
#include "us_radar_msgs/DistancesArray.h"
#include "us_radar_msgs/Detections.h"
#include "us_radar_msgs/Distances.h"
#include "usRadar.h"
#include "byte.h"

// cannet
#include "CanFrame.h"

// rviz dispaly
#include <visualization_msgs/MarkerArray.h>	

//headers in Autowae Health Checker
//#include <health_checker/node_status_publisher.h>

using namespace ros;
using namespace std;

string ipAddr = "192.168.110.101";
unsigned int ipPort = 4002;
string frameID = "us_radar";
//std::shared_ptr<health_checker::NodeStatusPublisher> node_status_pub_ptr;

#define BigLittleSwap16(A)        ((((uint16)(A) & 0xff00) >> 8) | (((uint16)(A) & 0x00ff) << 8))

/**
 *  * @brief radar enable/disable
 *   * 		frame id: 0x220
 *    * 		byte[0] bit[0-1]: 0x11:ON 0xff:OFF
 *     *
 *      * @param on enable/disble
 *       */

void radarOn(bool on)
{
  XUdp sxudp;
  uint8 usBuf[13];		//13 byte

  memset(usBuf, 0, sizeof(usBuf));
  usBuf[0] = 0x08; //0x08				//msg header
  usBuf[1] = 0x00;
  usBuf[2] = 0x00;
  usBuf[3] = 0x02; //Frame ID: 0x220 radar on-off control.
  usBuf[4] = 0x20;

  if (on)
  {
    usBuf[5] = 0x11; // radar ON
  }
  else
  {
    usBuf[5] = 0xff; // radar OFF
  }

  sxudp.Send(ipAddr.c_str(), ipPort, usBuf, 13);
}

/**
 *  * @brief fill message to publish
 *   *
 *    * @param d detection's msg to fill
 *    * @param m marker's msg to fill
 *     * @param pbuf data buffer
 *      * @param where rear radar: 0, front radar: 1
 *       */

void messageFill(us_radar_msgs::Detections &d,
                 visualization_msgs::Marker &m, uint8 *pbuf,
                 uint8 where)
{
  uint8 mapx;

  m.header.frame_id = frameID;               	//"ultrasonic_radar";
  m.header.stamp = ros::Time::now();
  m.id = where;
  m.type = visualization_msgs::Marker::SPHERE;
  m.pose.orientation.x = 0.0;
  m.pose.orientation.y = 0.0;
  m.pose.orientation.z = 0.0;
  m.pose.orientation.w = 0.0;
  m.scale.x = 0.2;
  m.scale.y = 0.2;
  m.scale.z = 0.2;
  m.color.a = 1.0;
  m.color.r = 0.0;
  m.color.g = 1.0;
  m.color.b = 0.0;

  Byte b0 = Byte(pbuf);
  Byte b1 = Byte(pbuf + 1);
  Byte b2 = Byte(pbuf + 2);
  Byte b3 = Byte(pbuf + 3);
  d.where = where; // rear:0 front:1
  d.judge = b0.get_byte(0, 2);

  uint8 cord_sort = b0.get_byte(2, 1);
  if(cord_sort != 0x11)	cord_sort = 0x00;			//filter others
  d.cord_sort = cord_sort;

  d.trace = b0.get_byte(3, 5);				//not used yet
  d.group = b1.get_byte(0, 4);

  mapx = b2.get_byte(0, 7);

  // map_x is range from 0 - 100 and maped to -5 metre to +5 metre in float
  if (mapx > 100 || mapx < 0)
    ROS_INFO("[us-radar]-mapx: %d, the value range should be 0 - 100.", mapx);
  if (mapx == 50)
  {
    d.map_x = 0.0;
  }
  else if (mapx < 50)
  {
    d.map_x = -((float)(50 - mapx) / 10);
  }
  else if (mapx > 50)
  {
    d.map_x = (float)(mapx - 50) / 10;
  }

  d.mapz_judge = b2.get_byte(7, 1);        			// High: 1, Low: 0
  d.map_y = (float)b3.get_byte(0, 6) / 10; 		// to metre. value * 10 / 100 = value / 10

  //ROS_INFO_STREAM("[us-radar] - mapx is: " << (unsigned short)mapx 
  //          << ", and map_float is : " << d.map_x );

  m.pose.position.x = d.map_x;
  m.pose.position.y = d.map_y;
  //m.pose.position.z = (float)d.mapz_judge;
  m.pose.position.z = 0.0;
}

int
main (int argc, char **argv)
{
  XUdp xudp;
  //XUdp sxudp;
  uint8 buf[1024] = { 0 };
  char ip[100] = { 0 };
  int len;

  init (argc, argv, "us_radar_node");
  NodeHandle n;

  ipAddr = getParam < string > ("us_radar/addr", "192.168.110.101");
  ipPort = getParam < int >("us_radar/port", 4002);
  frameID = getParam < string > ("us_radar/frameID", "us_radar");
 // node_status_pub_ptr = std::make_shared<health_checker::NodeStatusPublisher>(n);
 // node_status_pub_ptr->ENABLE();

  Publisher DetPub =
    n.advertise < us_radar_msgs::DetectionsArray > ("/us_radar/Detections", 1000);
  Publisher DistPub =
    n.advertise < us_radar_msgs::DistancesArray > ("/us_radar/Distances", 1000);
  Publisher MarkerPub =
      n.advertise<visualization_msgs::MarkerArray>("usrMarkerArray", 1000);	
	

  // Default is enabled, so we don't need it by now,
  // Us_radar can be turn on or off according to car's speed.
  //
  if(true)
    radarOn (true);			// TODO comment it after debug
  else
  {
	  radarOn (false);	
	  ROS_INFO("[us-radar]-radar off.");
  }

  if (xudp.Bind (ipPort) < 0){
	  ROS_ERROR("[us-radar]-UDP bind ioport failure.");
	  return -1;
  }    

  Rate loop_rate (10);		//10Hz
  while (ok ())
    {
      uint8 *p;
      uint8 mapX;

      memset (buf, 0, sizeof (buf));
      memset (ip, 0, sizeof (ip));

      len = xudp.Receive (buf, 1024, ip);

      int n = len / 13;
      p = buf;

      if (len % 13 == 0)
	{
	  while (n)
	    {
	      uint16 i, j;

	      int flag = 0;

		  us_radar_msgs::DetectionsArray detArray;
          us_radar_msgs::Detections det;

          visualization_msgs::MarkerArray markerArray;
          visualization_msgs::Marker marker;

	      TimestampedCanFrame Tframe;
	      Tframe.frame.id = (static_cast<uint16> (*(p + 3) << 8) + *(p + 4));
	      for (j = 0; j < 8; j++)
		  {
		  	Tframe.frame.data[j] = *(p + j + 5);
		  }
		  Byte b0(p+5);
		  Byte b1(p+6);
		  Byte b2(p+7);
		  Byte b3(p+8);
		  Byte b4(p+9);
		  Byte b5(p+10);
		  Byte b6(p+11);
		  Byte b7(p+12);
	      
	      switch (Tframe.frame.id)
		{
		case MAP_R_FRAME:			//0x110-auto rear radar detect objects
		  {
          detArray.detections.clear();
          markerArray.markers.clear();
          messageFill(det, marker, p + 5, 0);

//        node_status_pub_ptr->NODE_ACTIVATE();
//  	  node_status_pub_ptr->CHECK_MAX_VALUE("/value/twist",ts.twist.linear.x,2.2,3.3,4.4,"linear twist_cmd is too high");;

		  //detections
          detArray.detections.push_back(det);
          DetPub.publish(detArray);

          // markers
          markerArray.markers.push_back(marker);
          MarkerPub.publish(markerArray);

		   if ( b7.get_byte(7, 1))
		   	break;

          detArray.detections.clear();
          markerArray.markers.clear();
          messageFill(det, marker, p + 9, 0);

		  //detections
          detArray.detections.push_back(det);
          DetPub.publish(detArray);

          // markers
          markerArray.markers.push_back(marker);
          MarkerPub.publish(markerArray);
     	  break;
		  }
		case MAP_F_FRAME:		//0x111-auto front radar detect objects
		  {
          detArray.detections.clear();
          markerArray.markers.clear();
          messageFill(det, marker, p + 5, 1);

		  //detections
          detArray.detections.push_back(det);
          DetPub.publish(detArray);

          // markers
          markerArray.markers.push_back(marker);
          MarkerPub.publish(markerArray);

		  if ( b7.get_byte(7, 1))
		  	break;

          detArray.detections.clear();
          markerArray.markers.clear();
          messageFill(det, marker, p + 9, 1);

		  //detections
          detArray.detections.push_back(det);
          DetPub.publish(detArray);

          // markers
          markerArray.markers.push_back(marker);
          MarkerPub.publish(markerArray);

		  break;
		  }
		case DISTANCES_FRAME:			//0x112-distances for single radar and radar pairs
		  {
		    us_radar_msgs::DistancesArray distArray;
		    us_radar_msgs::Distances rDist;

		    distArray.distances.clear ();
		    rDist.channel = p[5];			//default value 0xff	
			  if(rDist.channel == 0xff){
				  //ROS_INFO("[us-radar]-distance channel %d: is default value.", rDist.channel);		
				  break;
			  }				
			  else if(rDist.channel > 7){
				  ROS_WARN("[us-radar]-distance channel %d: is out of range.", rDist.channel);	
				  break;
			  }				
	
		    rDist.single_dist = ((uint16) (*(p + 6) << 8) + *(p + 7));		//default value 0x199A
		    rDist.pair1_dist = ((uint16) (*(p + 8) << 8) + *(p + 9));		//default value 0x199A
		    rDist.pair2_dist = ((uint16) (*(p + 10) << 8) + *(p + 11));		//default value 0x199A
			if((rDist.single_dist == 0x199A) && (rDist.pair1_dist == 0x199A) && (rDist.pair2_dist == 0x199A)){
				//ROS_INFO("[us-radar]-channel %d: distances are default values.", rDist.channel);
				break;
			}
			uint8 single_dist_high = *(p + 6);
			if(single_dist_high > 31){
				ROS_INFO("[us-radar]-single_dist_high: %d is more than 31.", single_dist_high);
				single_dist_high = 31;
			}	
			uint8 pair1_dist_high = *(p + 8);
			if(pair1_dist_high > 31){
				ROS_INFO("[us-radar]-pair1_dist_high: %d is more than 31.", pair1_dist_high);
				pair1_dist_high = 31;
			}
			uint8 pair2_dist_high = *(p + 10);
			if(pair2_dist_high > 31){
				ROS_INFO("[us-radar]-pair2_dist_high: %d is more than 31.", pair2_dist_high);
				pair2_dist_high = 31;
			}

		    rDist.single_dist = ((uint16) (single_dist_high << 8) + *(p + 7));		//TODO: how to convert uint16 to float32(from uint16 to length)
		    rDist.pair1_dist = ((uint16) (pair1_dist_high << 8) + *(p + 9));		
		    rDist.pair2_dist = ((uint16) (pair2_dist_high << 8) + *(p + 11));		

		    distArray.distances.push_back (rDist);
		    DistPub.publish (distArray);

		    break;
		  }
		default:
		  break;
		}
	      n--;
	      p = p + 13;
	    }		//end of while (n)
	}		//end of if (len % 13 == 0)
      spinOnce();
      loop_rate.sleep();
    }		//end of while (ok ())

  return 0;
}
