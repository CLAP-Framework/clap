/*
 * @Descripttion:
 * @version:
 * @Author: xiangzhang
 * @Date: 2020-09-27 19:38:51
 * @LastEditors: xiangzhang
 * @LastEditTime: 2020-09-27 19:39:06
 */
#ifndef ICV_CARLA_RVIZ_H
#define ICV_CARLA_RVIZ_H

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/String.h>
#include <stdio.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>

#include <cmath>
#include <fstream>
#include <iostream>

namespace Control {

visualization_msgs::Marker pubVecilePosetoRviz(geometry_msgs::Pose msg);
visualization_msgs::Marker pubPrepointtoRviz(geometry_msgs::Pose msg);
void pubPathtoRviz(const nav_msgs::PathConstPtr &msg_path);

}  // namespace Control

#endif
