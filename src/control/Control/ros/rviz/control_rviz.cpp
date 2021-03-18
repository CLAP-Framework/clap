/*
 * @Descripttion:
 * @version:
 * @Author: xiangzhang
 * @Date: 2020-09-27 19:38:51
 * @LastEditors: xiangzhang
 * @LastEditTime: 2020-09-27 19:48:52
 */

#include "control_rviz.h"

using namespace std;
namespace Control {
extern ros::Publisher pub_path;
visualization_msgs::Marker pubVecilePosetoRviz(geometry_msgs::Pose msg) {
  // visualization of ego_pose
  visualization_msgs::Marker Vehicle_xp;
  Vehicle_xp.header.frame_id = "map";
  Vehicle_xp.header.stamp = ros::Time::now();
  Vehicle_xp.ns = "my_namespace";
  Vehicle_xp.id = 0;
  Vehicle_xp.type = visualization_msgs::Marker::CUBE;
  Vehicle_xp.action = visualization_msgs::Marker::ADD;
  Vehicle_xp.pose.position.x = msg.position.x;  // currentpose.x
  Vehicle_xp.pose.position.y = msg.position.y;  // currentpose.y
  Vehicle_xp.pose.position.z = 0;
  Vehicle_xp.pose.orientation.x = msg.orientation.x;  //
  Vehicle_xp.pose.orientation.y = msg.orientation.y;  //
  Vehicle_xp.pose.orientation.z = msg.orientation.z;  //
  Vehicle_xp.pose.orientation.w = msg.orientation.w;  //
  Vehicle_xp.scale.x = 1;
  Vehicle_xp.scale.y = 1.5;
  Vehicle_xp.scale.z = 1;
  Vehicle_xp.color.a = 1.0;  // Don't forget to set the alpha!
  Vehicle_xp.color.r = 1.0;
  Vehicle_xp.color.g = 1.0;
  Vehicle_xp.color.b = 0.0;

  // pub_vehi_pose.publish(Vehicle_xp);//publish data of G1_vehicle
  return Vehicle_xp;
}

visualization_msgs::Marker pubPrepointtoRviz(geometry_msgs::Pose msg_pre) {
  // visualization of ego_pose
  visualization_msgs::Marker pre_point;
  pre_point.header.frame_id = "map";
  pre_point.header.stamp = ros::Time::now();
  pre_point.ns = "my_namespace_pre";
  pre_point.id = 0;
  pre_point.type = visualization_msgs::Marker::CUBE;
  pre_point.action = visualization_msgs::Marker::ADD;
  pre_point.pose.position.x = msg_pre.position.x;  // currentpose.x
  pre_point.pose.position.y = msg_pre.position.y;  // currentpose.y
  pre_point.pose.position.z = 0;
  pre_point.pose.orientation.x = msg_pre.orientation.x;  //
  pre_point.pose.orientation.y = msg_pre.orientation.y;  //
  pre_point.pose.orientation.z = msg_pre.orientation.z;  //
  pre_point.pose.orientation.w = msg_pre.orientation.w;  //
  pre_point.scale.x = 0.4;
  pre_point.scale.y = 0.4;
  pre_point.scale.z = 0.4;
  pre_point.color.a = 1.0;  // Don't forget to set the alpha!
  pre_point.color.r = 1.0;
  pre_point.color.g = 0.0;
  pre_point.color.b = 0.0;

  // pub_vehi_pose.publish(Vehicle_xp);//publish data of G1_vehicle
  return pre_point;
}
}  // namespace Control
