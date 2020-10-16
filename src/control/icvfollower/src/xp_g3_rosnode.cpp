/*
 * @Author: your name
 * @Date: 2020-09-15 11:57:11
 * @LastEditTime: 2020-09-15 21:36:38
 * @LastEditors: xiangzhang
 * @Description: In User Settings Edit
 * @FilePath:
 * /2.control_gitlab/src/review_code/ros/rosnode/novauto_car/src/novauto_rosnode.cpp
 */

#include "ros/ros.h"
#include "xp_g3_ros_core.h"

using namespace Control;

int main(int argc, char **argv) {
  ros::init(argc, argv, "ControllerNode");
  xp_g3_ros_core cute;
  cute.control_run();

  return 0;
}
