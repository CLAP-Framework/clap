/*
 * @Descripttion:
 * @version:
 * @Author: xiangzhang
 * @Date: 2020-09-15 20:54:58
 * @LastEditors: xiangzhang
 * @LastEditTime: 2020-10-12 15:13:06
 */

#include "xp_g3_ros_core.h"

#include <fstream>
#include <iostream>

using namespace Control;

xp_g3_ros_core::xp_g3_ros_core(/* args */) : LOOP_RATE_(100) { RosInit(); }

xp_g3_ros_core::~xp_g3_ros_core() {}

void xp_g3_ros_core::RosInit()
{
  ros::param::get("~deadacc", deadacc);
  // setup subscriber
  // sub trajectory
  sub_zzz_path_ = private_nh_.subscribe("/zzz/planning/decision_trajectory", 10,
                                        &xp_g3_ros_core::callback_Path, this);
  // sub zzz egopose
  sub_zzz_eogpose_ = private_nh_.subscribe(
      "/zzz/navigation/ego_pose", 10, &xp_g3_ros_core::callback_egopose, this);
  // sub localization
  sub_imudata_ = private_nh_.subscribe("/localization/imu/data", 10,
                                       &xp_g3_ros_core::callback_imu, this);
  sub_gpsfix_ = private_nh_.subscribe("/localization/gps/fix", 10,
                                      &xp_g3_ros_core::callback_gpsfix, this);
  sub_gpsvel_ = private_nh_.subscribe("/localization/gps/vel", 10,
                                      &xp_g3_ros_core::callback_gpsvel, this);
  sub_gpsodom_ = private_nh_.subscribe("/localization/gps/odom", 10,
                                       &xp_g3_ros_core::callback_gpsodom, this);
  // sub xpg3_chassis chassis data
  sub_g3_scu1_ = private_nh_.subscribe("/canbus/SCU_IPC_1", 10,
                                       &xp_g3_ros_core::callback_scu1, this);
  sub_g3_scu2_ = private_nh_.subscribe("/canbus/SCU_IPC_2", 10,
                                       &xp_g3_ros_core::callback_scu2, this);
  sub_g3_scu3_ = private_nh_.subscribe("/canbus/SCU_IPC_3", 10,
                                       &xp_g3_ros_core::callback_scu3, this);
  sub_g3_scu4_ = private_nh_.subscribe("/canbus/SCU_IPC_4", 10,
                                       &xp_g3_ros_core::callback_scu4, this);
  sub_g3_scu5_ = private_nh_.subscribe("/canbus/SCU_IPC_5", 10,
                                       &xp_g3_ros_core::callback_scu5, this);
  sub_g3_scu6_ = private_nh_.subscribe("/canbus/SCU_IPC_6", 10,
                                       &xp_g3_ros_core::callback_scu6, this);
  // advertise cmd to chassis
  pub_ipc1_ = private_nh_.advertise<canbus_msgs::IPC_SCU_1_0x106>(
      "/canbus/IPC_SCU_1", 1000);
  pub_ipc2_ = private_nh_.advertise<canbus_msgs::IPC_SCU_2_0x102>(
      "/canbus/IPC_SCU_2", 1000);
  pub_ipc3_ = private_nh_.advertise<canbus_msgs::IPC_SCU_3_0x103>(
      "/canbus/IPC_SCU_3", 1000);
}
void xp_g3_ros_core::control_run()
{
  ROS_INFO_STREAM("icv_carla_control  start");
  ros::Rate loop_rate(LOOP_RATE_);
  double angle = 0;
  float accelera = 0;
  float hold_brake = 0.0;
  int cout_brake = 0;
  //
  data_file_input();

  while (ros::ok())
  {

    // latcontroller
    //INIT_OK
    bool INI_OK = false;
    if (callback_imu_flag == 0)
      ROS_WARN("gps not receive !!!!");
    if (callback_Path_flag == 0)
      ROS_WARN("path not receive !!!!");
    if (callback_scu1_flag == 0)
      ROS_WARN("chassis_data not receive !!!!");

    if (callback_imu_flag == 1 && callback_Path_flag == 1 && callback_scu1_flag == 1)
    {
      INI_OK = true;
    }
    if (INI_OK)
    {
      //控制算法接口
      ego_vehicle_controller.get_vehicle_status(ego_vehicle_status);
      ego_vehicle_controller.get_localization(ego_vehicle_pose);
      ego_vehicle_controller.maincontroller_run();
      ego_vehicle_controller.send_vehicle_instruction(&ego_vehicle_control);

      angle = ego_vehicle_control.steerangle_req; // xp left +  right -
      accelera = ego_vehicle_control.acceleration;

      ROS_INFO("XpMotors Vehicle SteerSend =  %lf", angle);
      ROS_INFO("XpMotors Vehicle accelera =  %lf", accelera);

      // brake_hold 916 zxadd
      // if (speed_plan_desir < 0.01 && SCU_IPC_VehSpd < 0.01)
      //   cout_brake++;
      // else
      //   cout_brake = 0;
      // if (cout_brake / LOOP_RATE_ > 3)
      //   hold_brake = 1;
      // else
      //   hold_brake = 0;
      // 2020.9.22 G3_steer angle boudary by zhangxiang
      float SCU_IPC_VehSpd_km = SCU_IPC_VehSpd * 3.6;
      float steerangle_max = 0;
      if (SCU_IPC_VehSpd_km <= 15)
        steerangle_max = 507;
      if (SCU_IPC_VehSpd_km > 15 && SCU_IPC_VehSpd_km <= 25)
        steerangle_max = 300;
      if (SCU_IPC_VehSpd_km > 25 && SCU_IPC_VehSpd_km <= 35)
        steerangle_max = 110;
      if (SCU_IPC_VehSpd_km > 35 && SCU_IPC_VehSpd_km <= 45)
        steerangle_max = 60;
      if (SCU_IPC_VehSpd_km > 45 && SCU_IPC_VehSpd_km <= 55)
        steerangle_max = 40;
      if (angle < -steerangle_max)
        angle = -steerangle_max;
      if (angle > steerangle_max)
        angle = steerangle_max;
    }
    else
    {
      accelera = 0;
      angle = 0;
      ROS_ERROR("ERROR GPS OR PALNNING MODULE!!!");
    }
    // publish control instructions
    publish_control_instruction(accelera, angle, hold_brake);
    // for visualization with Rviz
    pubVechilepose2rviz(ego_vehicle_pose);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
void xp_g3_ros_core::publish_control_instruction(float out_acc, float out_steer,
                                                 int brake_hold)
{
  canbus_msgs::IPC_SCU_1_0x106 g3_msg1;
  canbus_msgs::IPC_SCU_2_0x102 g3_msg2;
  canbus_msgs::IPC_SCU_3_0x103 g3_msg3;
  //车门安全检测
  int vehicle_door_close_flag = 0;

  if (SCU_IPC_DriverDoorAjarSt == 0 && SCU_IPC_PsngrDoorAjarSt == 0 &&
      SCU_IPC_RLDoorAjarSt == 0 && SCU_IPC_RRDoorAjarSt == 0)
    vehicle_door_close_flag = 1;
  else
    ROS_WARN("Please check vehicle door !!!! not close");

  if (drive_mode == 3 || SCU_IPC_BrkPedalSt == 1 && SCU_IPC_BrkPedalStVD == 1)
    RESTART = 0;
  if (SCU_IPC_HighBeamOutputSt == 1)
    RESTART = 1;

  if (vehicle_door_close_flag == 1 && RESTART == 1 && SCU_IPC_BrkPedalSt != 1 &&
      (drive_mode == 2 || drive_mode == 3))
  {
    g3_msg1.IPC_SCU_DBWReq = 1;
    g3_msg1.IPC_SCU_ReadyReq = 1;
  }
  else
  {
    g3_msg1.IPC_SCU_ReadyReq = 0;
    g3_msg1.IPC_SCU_DBWReq = 3;

    g3_msg1.IPC_SCU_TorsionBarTqReq = 0;
    g3_msg1.IPC_SCU_TorsionBarTqReqVD = 0;
  }
  // gear deal  g3_msg1.IPC_SCU_GearReq = 2; //1n2d3r4p
  int gear_req = 2;
  g3_msg2.IPC_SCU_ParkingReqToEPB = 1;   // RELEASE epb
  g3_msg2.IPC_SCU_ParkingReqToEPBVD = 1; // RELEASE epb
  if (brake_hold == 1)
  {
    gear_req = 4;
    g3_msg2.IPC_SCU_ParkingReqToEPB = 0;
    g3_msg2.IPC_SCU_ParkingReqToEPBVD = 1;
  }
  g3_msg1.IPC_SCU_GearReq = gear_req;
  g3_msg1.IPC_SCU_SteerAngleReq = out_steer;
  g3_msg1.IPC_SCU_SteerAngleReqVD = 1;
  g3_msg2.IPC_SCU_AccDecelReq = 0;
  g3_msg2.IPC_SCU_AccDecelReqVD = 0;
  if (SCU_IPC_EPBSysSt == 0 && SCU_IPC_CurrentGearLev == gear_req)
  {
    g3_msg2.IPC_SCU_AccDecelReq = out_acc;
    g3_msg2.IPC_SCU_AccDecelReqVD = 1;
  }
  g3_msg2.IPC_SCU_MotorTorqReq = 0;
  g3_msg2.IPC_SCU_MotorTorqReqVD = 0;

  // g3_msg2.IPC_SCU_VehSpdVD = brake_hold; //hold vehilcle
  pub_ipc1_.publish(g3_msg1);
  pub_ipc2_.publish(g3_msg2);
}
// pub positon and prepoint to rviz
void xp_g3_ros_core::pubVechilepose2rviz(
    const control_msgs::Pathpoint &vehicle_pose)
{
  geometry_msgs::Pose carpoint, pre_point;
  carpoint.position.x = vehicle_pose.x;
  carpoint.position.y = vehicle_pose.y;
  geometry_msgs::Quaternion quaternion;
  quaternion = tf::createQuaternionMsgFromRollPitchYaw(0, 0, vehicle_pose.yaw);
  carpoint.orientation = quaternion;
  visualization_msgs::Marker Vehicle_msg = pubVecilePosetoRviz(carpoint);
  pub_vehi_pose.publish(Vehicle_msg);

  // double x_pre, y_pre;
  // float d_error;
  // zz.sendXYpre(&x_pre, &y_pre, &d_error);
  // pre_point.position.x = x_pre;
  // pre_point.position.y = y_pre;
  // visualization_msgs::Marker pre_point_msg = pubPrepointtoRviz(pre_point);

  // pub_pre_point.publish(pre_point_msg);
}

void xp_g3_ros_core::callback_egopose(
    const zzz_driver_msgs::RigidBodyStateStamped &msg)
{
  ego_vehicle_pose.x = msg.state.pose.pose.position.x;
  ego_vehicle_pose.y = msg.state.pose.pose.position.y;
  ego_vehicle_pose.yaw = qua_to_rpy(msg.state.pose.pose.orientation);
  callback_imu_flag = true;
}

void xp_g3_ros_core::callback_Path(const zzz_planning_msgs::DecisionTrajectory &msg)
{
  // maincontroller
  vector<control_msgs::Trajectorypoint> local_trajectory;
  local_trajectory.clear();
  control_msgs::Trajectorypoint local_trajectoryPoint;
  if (msg.trajectory.poses.size() > 2)
  {
    for (int i = 0; i < msg.trajectory.poses.size(); ++i)
    {
      local_trajectoryPoint.pose.x = msg.trajectory.poses[i].pose.position.x;
      local_trajectoryPoint.pose.y = msg.trajectory.poses[i].pose.position.y;
      if (i < msg.desired_speed.size())
      {
        local_trajectoryPoint.twist.velocity = msg.desired_speed[i];
      }
      local_trajectoryPoint.pose.curvature = 0;
      if (i < msg.kappa.size())
      {
        local_trajectoryPoint.pose.curvature = msg.kappa[i];
      }
      local_trajectory.push_back(local_trajectoryPoint);
    }
    ego_vehicle_controller.get_trajectory(local_trajectory);
  }
  else
  {
    ROS_WARN(" WARNING : trajectory  size error !!!! path size =  %ld  desired_speed size =  %ld  kappa size =  %ld",
             msg.trajectory.poses.size(), msg.desired_speed.size(), msg.kappa.size());
  }

  // cout << "local_trajectory  " << local_trajectory.size() << endl;
  ROS_DEBUG("I hear pointsize %ld", msg.trajectory.poses.size());
  callback_Path_flag = true;
}

void xp_g3_ros_core::callback_gpsvel(const geometry_msgs::TwistWithCovarianceStamped &msg)
{
  //  state.state.twist.twist.linear.x=msg.twist.twist.linear.x;
  //  state.state.twist.twist.linear.y=msg.twist.twist.linear.y;
  //  state.state.twist.twist.linear.z=0;
  ROS_INFO_ONCE("I hear /gps/vel topic ~~~~~~");
}

void xp_g3_ros_core::callback_imu(const sensor_msgs::Imu &msg)
{
  double Yaw = 0;

  Imu_angle_acc_x = msg.angular_velocity.x;
  Imu_angle_acc_y = msg.angular_velocity.y;
  Imu_angle_acc_z = msg.angular_velocity.z;

  Imu_linear_accX = msg.linear_acceleration.x;
  Imu_linear_accY = msg.linear_acceleration.y;
  Imu_linear_accZ = msg.linear_acceleration.z;

  callback_imu_flag = true;
}

void xp_g3_ros_core::callback_gpsfix(const sensor_msgs::NavSatFix &msg)
{
  latitude = msg.latitude;
  longitude = msg.longitude;
  altitude = msg.altitude;

  callback_gpsfix_flag = true;
  ROS_INFO_ONCE("I hear /gps/fix topic ~~~~~~");
}
// receive UTM coordinate from localization
void xp_g3_ros_core::callback_gpsodom(const nav_msgs::Odometry &msg)
{
  // ego_vehicle_pose.x = msg.pose.pose.position.x;
  // ego_vehicle_pose.y = msg.pose.pose.position.y;
  // ego_vehicle_pose.yaw = qua_to_rpy(msg.pose.pose.orientation);  // rad

  callack_odom_flag = true;
  ROS_INFO_ONCE("I hear /gps/odom topic ~~~~~~");
}
// chassis callback
void xp_g3_ros_core::callback_scu1(const canbus_msgs::SCU_IPC_1_0x20A &msg)
{
  SCU_IPC_SteeringAngleVD = msg.SCU_IPC_SteeringAngleVD;
  SCU_IPC_SteeringAngleSpd = msg.SCU_IPC_SteeringAngleSpd;
  SCU_IPC_SteeringAngle = msg.SCU_IPC_SteeringAngle;
  SCU_IPC_ResponseTorque = msg.SCU_IPC_ResponseTorque;
  SCU_IPC_ResponseTorqueVD = msg.SCU_IPC_ResponseTorqueVD;
  SCU_IPC_SteeringAngleSpdVD = msg.SCU_IPC_SteeringAngleSpdVD;
  // maincocntroller
  if (SCU_IPC_SteeringAngleVD)
  {
    ego_vehicle_status.steerangle_fd = SCU_IPC_SteeringAngle;
    SCU_IPC_SteeringAngle_last = SCU_IPC_SteeringAngle;
  }
  else
  {
    ego_vehicle_status.steerangle_fd = SCU_IPC_SteeringAngle_last;
  }
  ego_vehicle_status.steeranglespeed_fd = SCU_IPC_SteeringAngleSpd;

  callback_scu1_flag = true;
}

void xp_g3_ros_core::callback_scu2(const canbus_msgs::SCU_IPC_2_0x205 &msg) {}

void xp_g3_ros_core::callback_scu3(const canbus_msgs::SCU_IPC_3_0x206 &msg)
{
  drive_mode = msg.SCU_IPC_DBWSt;
  SCU_IPC_BrkPedalSt = msg.SCU_IPC_BrkPedalSt;
  SCU_IPC_BrkPedalStVD = msg.SCU_IPC_BrkPedalStVD;
  SCU_IPC_VehSpd = msg.SCU_IPC_VehSpd / 3.6; // m/s
  SCU_IPC_BrkLightOn = msg.SCU_IPC_BrkLightOn;
  // maincocntroller
  ego_vehicle_status.velocity_fd = msg.SCU_IPC_VehSpd / 3.6; // m/s
  if (drive_mode == 6 || drive_mode == 7)
  {
    ego_vehicle_status.automode = 1;
  }
  else
  {
    ego_vehicle_status.automode = 0;
  }

  // SCU_IPC_3_MsgCounter = msg.SCU_IPC_3_MsgCounter;
  // SCU_IPC_3_Checksum = msg.SCU_IPC_3_Checksum;
}
void xp_g3_ros_core::callback_scu4(const canbus_msgs::SCU_IPC_4_0x207 &msg)
{
  vehicle_acc_x = msg.SCU_IPC_ActVehLongAccel;
  SCU_IPC_EPBSysSt = msg.SCU_IPC_EPBSysSt;
  ego_vehicle_status.acceleration_fd = vehicle_acc_x * 9.8;
}
void xp_g3_ros_core::callback_scu5(const canbus_msgs::SCU_IPC_5_0x208 &msg)
{
  SCU_IPC_DriverDoorLockSt = msg.SCU_IPC_DriverDoorLockSt;
  SCU_IPC_DriverDoorAjarSt = msg.SCU_IPC_DriverDoorAjarSt;
  SCU_IPC_PsngrDoorAjarSt = msg.SCU_IPC_PsngrDoorAjarSt;
  SCU_IPC_RLDoorAjarSt = msg.SCU_IPC_RLDoorAjarSt;
  SCU_IPC_RRDoorAjarSt = msg.SCU_IPC_RRDoorAjarSt;
  SCU_IPC_LTurnLampOutputSt = msg.SCU_IPC_LTurnLampOutputSt;
  SCU_IPC_RTurnLampOutputSt = msg.SCU_IPC_RTurnLampOutputSt;
  SCU_IPC_HazardLampOutputSt = msg.SCU_IPC_HazardLampOutputSt;
  SCU_IPC_LowBeamOutputSt = msg.SCU_IPC_LowBeamOutputSt;
  SCU_IPC_HighBeamOutputSt = msg.SCU_IPC_HighBeamOutputSt;
  SCU_IPC_Horndriverst = msg.SCU_IPC_Horndriverst;
  SCU_IPC_FrontWiperOutputSt = msg.SCU_IPC_FrontWiperOutputSt;
  SCU_IPC_PowerMode = msg.SCU_IPC_PowerMode;
}
void xp_g3_ros_core::callback_scu6(const canbus_msgs::SCU_IPC_6_0x209 &msg)
{
  if (msg.SCU_IPC_CurrentGearLevVD)
  {
    if( msg.SCU_IPC_CurrentGearLev == 1)
      SCU_IPC_CurrentGearLev = 2;
    if( msg.SCU_IPC_CurrentGearLev == 2)
      SCU_IPC_CurrentGearLev = 1;
    if( msg.SCU_IPC_CurrentGearLev == 3)
      SCU_IPC_CurrentGearLev = 3;
    if( msg.SCU_IPC_CurrentGearLev == 4)
      SCU_IPC_CurrentGearLev = 4;
  }

}

// need to replace
double xp_g3_ros_core::qua_to_rpy(geometry_msgs::Quaternion posedata)
{
  float w = posedata.w;
  float x = posedata.x;
  float y = posedata.y;
  float z = posedata.z;

  float R = atan2((2 * (w * x + y * z)), (1 - 2 * (x * x + y * y)));
  float P = asin(2 * (w * y - z * x));
  float Y = atan2((2 * (w * z + x * y)), (1 - 2 * (z * z + y * y)));

  return Y;
}
void xp_g3_ros_core::data_file_input()
{
  nav_msgs::Path Path_file;
  Path_file.poses.clear();
  geometry_msgs::PoseStamped Point_file;

  //读取采集到的路径点信息
  ifstream infile1;
  infile1.open("/home/fangniuwa/g3bug_fix/control_fix/src/Control/Path_control.txt"); //打开文件
  if (!infile1.good())
  {
    ROS_ERROR("no Path file !!!please check your pathfile!!!!!");
  }
  cout << " find file !!!! " << endl;
  char c;
  char line[256];
  int lineCnt = 0;

  vector<control_msgs::Trajectorypoint> local_trajectory;
  local_trajectory.clear();
  while (infile1.getline(line, 256, '\n'))
  {
    control_msgs::Trajectorypoint local_trajectory_point;
    sscanf(line, "%f %f %f",
           &(local_trajectory_point.pose.x),
           &(local_trajectory_point.pose.y),
           &(local_trajectory_point.pose.yaw));
    // cout << "point { " << local_trajectory_point.pose.x << ", y " << local_trajectory_point.pose.y
    // << ", yaw " << local_trajectory_point.pose.yaw << endl;
    local_trajectory_point.pose.yaw = local_trajectory_point.pose.yaw / 180 * M_PI;
    local_trajectory.push_back(local_trajectory_point);
    Point_file.pose.position.x = local_trajectory_point.pose.x;
    Point_file.pose.position.y = local_trajectory_point.pose.y;
    Path_file.poses.push_back(Point_file);
  }
  infile1.close();
  cout << " OPEN file successed !!!! " << endl;
  Path_file.header.frame_id = "map";

  ego_vehicle_controller.get_trajectory(local_trajectory);
  // cout<<" local_trajectory  size  "<<local_trajectory.size()<<endl;
  callback_Path_flag = true;
}
