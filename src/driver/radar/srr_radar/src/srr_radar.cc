#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

#include "srr_radar/frame/canet/canet.h"
#include "srr_radar/srr_radar_driver_ros.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "srr_radar");
  ros::NodeHandle n;
  // ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  ros::Rate loop_rate(10);    //10Hz

  drivers::SrrRadarDriverRos drv;
  drv.init(&n);
  drv.run();
  while (ros::ok())
  {
	  ros::spin();
  }
  return 0;
}

