#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

#include "continental_radar/frame/canet/canet.h"
#include "continental_radar/conti_radar_driver_ros.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "continental_radar");
  ros::NodeHandle n;
  // ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  ros::Rate loop_rate(10);

  int count = 0;
  drivers::ContiRadarDriverRos drv;

  drv.init(&n);
  drv.run();
  while (ros::ok())
  {
	  ros::spin();
  }

  return 0;
}

