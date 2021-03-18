/*
 * @Descripttion:
 * @version:
 * @Author: xiangzhang
 * @Date: 2020-09-15 20:55:17
 * @LastEditors: xiangzhang
 * @LastEditTime: 2020-10-12 14:28:18
 */
#pragma once

#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

#include "../../../../controller/maincontroller/maincontroller.h"
#include "../../../../controller/pid_lon_controller/loncontrol.h"
#include "../../../../controller/pre_lat_controller/pre_lat_controller.h"
#include "../../../rviz/control_rviz.h"
#include "autoware_msgs/Lane.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "ros/ros.h"
#include "zzz_driver_msgs/RigidBodyStateStamped.h"
#include "zzz_planning_msgs/DecisionTrajectory.h"

#include "canbus_msgs/IPC_SCU_1_0x106.h"
#include "canbus_msgs/IPC_SCU_2_0x102.h"
#include "canbus_msgs/IPC_SCU_3_0x103.h"
#include "canbus_msgs/SCU_IPC_1_0x20A.h"
#include "canbus_msgs/SCU_IPC_2_0x205.h"
#include "canbus_msgs/SCU_IPC_3_0x206.h"
#include "canbus_msgs/SCU_IPC_4_0x207.h"
#include "canbus_msgs/SCU_IPC_5_0x208.h"
#include "canbus_msgs/SCU_IPC_6_0x209.h"
#include "canbus_msgs/SCU_IPC_7_0x301.h"

namespace Control
{

  class xp_g3_ros_core
  {
  private:
    // main controller test
    maincontroller ego_vehicle_controller;
    // control
    control_msgs::Vehicle_control ego_vehicle_control;
    control_msgs::Vehicle_status ego_vehicle_status;
    control_msgs::Pathpoint ego_vehicle_pose;

    // ros paraeters init
    ros::NodeHandle private_nh_;
    ros::NodeHandle rviz_n;

    // define publisher
    ros::Publisher pub_ipc1_, pub_ipc2_, pub_ipc3_;
    // pub rviz
    ros::Publisher pub_vehi_pose =
        rviz_n.advertise<visualization_msgs::Marker>("/dobot_maker1", 1000);
    ros::Publisher pub_pre_point =
        rviz_n.advertise<visualization_msgs::Marker>("/pre_point", 1000);

    /*chassis parameters of function topic*/
    // sub chassis data
    ros::Subscriber sub_g3_scu1_, sub_g3_scu2_, sub_g3_scu3_, sub_g3_scu4_,
        sub_g3_scu5_, sub_g3_scu6_;
    // sub localization
    ros::Subscriber sub_imudata_, sub_gpsfix_, sub_gpsvel_, sub_gpsodom_;
    // sub zzz
    ros::Subscriber sub_zzz_path_, sub_zzz_eogpose_;

    const int LOOP_RATE_;
    int RESTART = 0;
    // callback parameters of chassis datas
    float deadacc = 0;
    float target_v = 0; // m/s
    float speed_plan_desir = 0;
    float speed_plan_desir_old = 0;
    //
    float Imu_angle_acc_x = 0;
    float Imu_angle_acc_y = 0;
    float Imu_angle_acc_z = 0;

    float Imu_linear_accX = 0;
    float Imu_linear_accY = 0;
    float Imu_linear_accZ = 0;
    // gps fix
    float latitude = 0;
    float longitude = 0;
    float altitude = 0;
    float glo_x = 0;
    float glo_y = 0;
    float glo_yaw = 0;
    // safety flag
    int gear_flag = 0;
    bool callback_imu_flag = true;
    bool callback_gpsfix_flag = true;
    bool vehicle_status_flag = false;
    bool callack_odom_flag = false;
    bool callback_Path_flag = false;
    bool callback_scu1_flag = false;

    // chassis parameters
    int drive_mode = 0;
    float vehicle_acc_x = 0;
    float SCU_IPC_VehSpd = 0.0;
    float vehicle_speed = 0.0;
    float SCU_IPC_BrkLightOn = 0;
    float SCU_IPC_BrkPedalSt = 0;
    float SCU_IPC_BrkPedalStVD = 0;
    float SCU_IPC_EPBSysSt = 0;
    float SCU_IPC_SteeringAngleVD = 0.0;
    float SCU_IPC_SteeringAngleSpd = 0.0;
    float SCU_IPC_SteeringAngle = 0.0;
    float SCU_IPC_SteeringAngle_last = 0.0;
    float SCU_IPC_ResponseTorque = 0.0;
    float SCU_IPC_ResponseTorqueVD = 0.0;
    float SCU_IPC_SteeringAngleSpdVD = 0.0;
    float steeringAngle = 0.0;

    // SCU_IPC_8
    int SCU_IPC_DriverDoorLockSt = 0;
    int SCU_IPC_DriverDoorAjarSt = 0;
    int SCU_IPC_PsngrDoorAjarSt = 0;
    int SCU_IPC_RLDoorAjarSt = 0;
    int SCU_IPC_RRDoorAjarSt = 0;
    int SCU_IPC_LTurnLampOutputSt = 0;
    int SCU_IPC_RTurnLampOutputSt = 0;
    int SCU_IPC_HazardLampOutputSt = 0;
    int SCU_IPC_LowBeamOutputSt = 0;
    int SCU_IPC_HighBeamOutputSt = 0;
    int SCU_IPC_Horndriverst = 0;
    int SCU_IPC_FrontWiperOutputSt = 0;
    int SCU_IPC_PowerMode = 0;

    //SCU_IPC_9
    int SCU_IPC_CurrentGearLev = 0;
    // funtions
    void RosInit();

    // finalwaypoint calllback funtions
    void callbackFromWayPoints(const autoware_msgs::LaneConstPtr &msg);
    // limit parameter max and min
    double LimitParameter(double parameter, double param_max, double param_min);

    // localization
    void callback_gpsvel(const geometry_msgs::TwistWithCovarianceStamped &msg);
    void callback_imu(const sensor_msgs::Imu &msg);
    void callback_gpsfix(const sensor_msgs::NavSatFix &msg);
    void callback_gpsodom(const nav_msgs::Odometry &msg);

    // chassis
    void callback_scu1(const canbus_msgs::SCU_IPC_1_0x20A &msg);
    void callback_scu2(const canbus_msgs::SCU_IPC_2_0x205 &msg);
    void callback_scu3(const canbus_msgs::SCU_IPC_3_0x206 &msg);
    void callback_scu4(const canbus_msgs::SCU_IPC_4_0x207 &msg);
    void callback_scu5(const canbus_msgs::SCU_IPC_5_0x208 &msg);
    void callback_scu6(const canbus_msgs::SCU_IPC_6_0x209 &msg);

    // get trajectory zzz
    void callback_Path(const zzz_planning_msgs::DecisionTrajectory &msg);
    void callback_egopose(const zzz_driver_msgs::RigidBodyStateStamped &msg);

    // update input data
    void vehiclestatusupdate();
    void vehiclecontrolupdate();
    double qua_to_rpy(geometry_msgs::Quaternion posedata);
    void publish_control_instruction(float out_acc, float out_steer,
                                     int brake_hold);
    void pubVechilepose2rviz(const control_msgs::Pathpoint &vehicle_pose);

  public:
    xp_g3_ros_core(/* args */);
    ~xp_g3_ros_core();
    void control_run();

    void data_file_input();
  };

} // namespace Control
