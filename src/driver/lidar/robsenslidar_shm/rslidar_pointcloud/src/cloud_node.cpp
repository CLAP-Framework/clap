/*
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2016 Robosense, Tony Zhang
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file

    This ROS node converts raw RSLIDAR LIDAR packets to PointCloud2.

*/
#include <ros/ros.h>
#include "convert.h"
#include "std_msgs/String.h"

volatile sig_atomic_t flag = 1;

static void my_handler(int sig)
{
  flag = 0;
}

/** Main node entry point. */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "cloud_node");
  ros::NodeHandle node;
  ros::NodeHandle priv_nh("~");

  //signal(SIGINT, my_handler);
// create 3 threads for 3 lidars scanMsg and 3 threads for difopMsg
  rslidar_pointcloud::Convert cloud_middle(node, priv_nh);      //middle used as for all
  boost::thread middle_thread_difop(boost::bind(&rslidar_pointcloud::Convert::difop_thread,&cloud_middle));

  /*rslidar_pointcloud::Convert cloud_left(node, priv_nh);
  boost::thread left_thread_msop(boost::bind(&rslidar_pointcloud::Convert::msop_thread,&cloud_left));
  boost::thread left_thread_difop(boost::bind(&rslidar_pointcloud::Convert::difop_thread,&cloud_left)); 

  rslidar_pointcloud::Convert cloud_right(node, priv_nh);
  boost::thread right_thread_msop(boost::bind(&rslidar_pointcloud::Convert::msop_thread,&cloud_right));
  boost::thread right_thread_difop(boost::bind(&rslidar_pointcloud::Convert::difop_thread,&cloud_right));
*/
  cloud_middle.msop_thread();    //middle msop thread

  middle_thread_difop.join();

  /*left_thread_msop.join();
  left_thread_difop.join();
  right_thread_msop.join();
  right_thread_difop.join();  */

  return 0;
}


