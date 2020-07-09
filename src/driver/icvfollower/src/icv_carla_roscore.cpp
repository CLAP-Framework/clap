
#include "icv_carla_node.h"

namespace icv
{
// Constructor
icvCarlaControlNode::icvCarlaControlNode()
    : LOOP_RATE_(20)
{
  initForROS();
}

// Destructor
icvCarlaControlNode::~icvCarlaControlNode()
{
}

void icvCarlaControlNode::initForROS()
{
//   ros parameter settings
#ifdef __DEBUG__SURPPORT_
  ros::param::get("~target_speedcarla", target_speedcarla);
  speed_plan_desir = target_speedcarla / 3.6;
  ros::param::get("~_R_test", _R_test);
  ros::param::get("~_delta_d", _delta_d);
#endif
  ros::param::get("~VehicleMode_flag", VehicleMode_flag);
  if (VehicleMode_flag == 1)
  {
#ifdef __DEBUG__SURPPORT_
    /**carla simulition**/
    // setup subscriber
    sub1_ = nh_.subscribe("/carla/ego_vehicle/waypoints", 10, &icvCarlaControlNode::WaypointCallback, this);
    sub2_ = nh_.subscribe("/gnss_pose", 10, &icvCarlaControlNode::callbackFromCurrentPose, this);
    sub4_ = nh_.subscribe("/carla/ego_vehicle/odometry", 10, &icvCarlaControlNode::callbackFromodom, this);

    sub3_ = nh_.subscribe("/carla/ego_vehicle/vehicle_status", 10, &icvCarlaControlNode::VehicleStatusCallback, this);
    // setup publisher
    pub1_ = nh_.advertise<carla_msgs::CarlaEgoVehicleControl>("/carla/ego_vehicle/vehicle_control_cmd", 10);
#endif
  }
  if (VehicleMode_flag == 2)
  {
    /**mini_auto simulition**/
    // setup subscriber
  #if 0
    sub1_ = nh_.subscribe("final_waypoints", 10, &icvCarlaControlNode::callbackFromWayPoints, this);
    sub2_ = nh_.subscribe("current_pose", 10, &icvCarlaControlNode::callbackFromCurrentPose, this);
    sub3_ = nh_.subscribe("/carla/ego_vehicle/vehicle_status", 10, &icvCarlaControlNode::VehicleStatusCallback, this);
  #endif

#ifdef __ZZZ_SURPPORT__
    sub_zzz_path_ = nh_.subscribe("/zzz/planning/decision_trajectory", 10, &icvCarlaControlNode::callback_Path, this);
#endif
    // setup publisher
    pub1_ = nh_.advertise<carla_msgs::CarlaEgoVehicleControl>("/carla/ego_vehicle/vehicle_control_cmd", 10);
  }

  if (VehicleMode_flag == 3)
  {
    /*小鹏实车调试使用*/
    sub_zzz_eogpose_ = nh_.subscribe("/zzz/navigation/ego_pose", 10, &icvCarlaControlNode::callback_egopose, this);
    // sub_imudata_ = nh_.subscribe("/imu/data", 10, &icvCarlaControlNode::callback_imu, this);
    // sub_gpsfix_ = nh_.subscribe("/gps/fix", 10, &icvCarlaControlNode::callback_gpsfix, this);
    // sub_gpsvel_ = nh_.subscribe("/gps/vel", 10, &icvCarlaControlNode::callback_gpsvel, this);
    sub_autostateex_ = nh_.subscribe("/xp/auto_state_ex", 10, &icvCarlaControlNode::callback_auto_state_ex, this);
    sub_autostate_ = nh_.subscribe("/xp/auto_state", 10, &icvCarlaControlNode::callback_auto_state, this);
    sub_esc_status_ = nh_.subscribe("/xp/esc_status", 10, &icvCarlaControlNode::callback_esc, this);
    sub_eps_status_ = nh_.subscribe("/xp/eps_status", 10, &icvCarlaControlNode::callback_eps, this);
#ifdef __ZZZ_SURPPORT__
    sub_zzz_path_ = nh_.subscribe("/zzz/planning/decision_trajectory", 10, &icvCarlaControlNode::callback_Path, this);
#endif

    pub_xp_ = nh_.advertise<xpmotors_can_msgs::AutoCtlReq>("/xp/auto_control", 1000);
  }

  if (VehicleMode_flag == 4)
  {
    /*园区小车调试使用*/
  #if 0
    sub1_ = nh_.subscribe("final_waypoints", 10, &icvCarlaControlNode::callbackFromWayPoints, this);
  #endif
    // sub2_ = nh_.subscribe("current_pose", 10, &icvCarlaControlNode::callbackFromCurrentPose, this);

    sub_imudata_ = nh_.subscribe("/localization/imu/data", 10, &icvCarlaControlNode::callback_imu, this);
    sub_gpsfix_ = nh_.subscribe("/localization/gps/fix", 10, &icvCarlaControlNode::callback_gpsfix, this);
    sub_gpsvel_ = nh_.subscribe("/localization/gps/vel", 10, &icvCarlaControlNode::callback_gpsvel, this);

    sub_gpsodom_ = nh_.subscribe("/localization/gps/odom", 10, &icvCarlaControlNode::callback_gpsodom, this);

    sub_autostateex_ = nh_.subscribe("/xp/auto_state_ex", 10, &icvCarlaControlNode::callback_auto_state_ex, this);
    sub_autostate_ = nh_.subscribe("/xp/auto_state", 10, &icvCarlaControlNode::callback_auto_state, this);
    sub_esc_status_ = nh_.subscribe("/xp/esc_status", 10, &icvCarlaControlNode::callback_esc, this);
    sub_eps_status_ = nh_.subscribe("/xp/eps_status", 10, &icvCarlaControlNode::callback_eps, this);
#ifdef __ZZZ_SURPPORT__
    sub_zzz_path_ = nh_.subscribe("/zzz/planning/decision_trajectory", 10, &icvCarlaControlNode::callback_Path, this);
#endif

    pub_xp_ = nh_.advertise<xpmotors_can_msgs::AutoCtlReq>("/xp/auto_control", 1000);
  }

  if (VehicleMode_flag != 1 && VehicleMode_flag != 2 && VehicleMode_flag != 3)
  {
    ROS_ERROR("VehicleMode_flag set error !!!!!!! please set Vehicleflag=2 for mini_auto system, set Vehicleflag =3 for xiaopeng or Vehicleflag = 4 for cutecar!!!!!");
  }
}
//传递carla信息
void icvCarlaControlNode::callbackFromodom(const nav_msgs::Odometry &msg)
{
  zz.setcarlaomodom(msg);
}

// void icvCarlaControlNode::callback_gpsodom(const nav_msgs::Odometry &msg)
// {

//   glo_x = msg.pose.pose.position.x;
//   glo_y = msg.pose.pose.position.y;
//   glo_yaw = qua_to_rpy(msg.pose.pose.orientation);
//   XpPoint_rec.theta = glo_yaw;
//   XpPoint_rec.x = glo_x;
//   XpPoint_rec.y = glo_y;
// }

void icvCarlaControlNode::callbackFromCurrentVelocity(const geometry_msgs::TwistStampedConstPtr &msg)
{
  XpPoint_rec.speed_rec = msg->twist.linear.x; //m/s
}

#if 0
void icvCarlaControlNode::callbackFromWayPoints(const autoware_msgs::LaneConstPtr &msg)
{
  nav_msgs::Path Path_dev; //回调函数接受final waypoins、
  geometry_msgs::PoseStamped pose_dev;
  if (!msg->waypoints.empty())
    target_speedcarla = msg->waypoints.at(0).twist.twist.linear.x;
  else
    target_speedcarla = 0;
  for (int i = 0; i < msg->waypoints.size(); i++)
  {
    pose_dev.pose.orientation = msg->waypoints[i].pose.pose.orientation;
    pose_dev.pose.position = msg->waypoints[i].pose.pose.position;
    Path_dev.poses.push_back(pose_dev);
  }

  speed_plan_desir = msg->waypoints[3].twist.twist.linear.x;
  zz.setWaypoints(Path_dev);
  ROS_INFO("I hear waypoint %ld", msg->waypoints.size());
  callback_Path_flag = true;
}
#endif

bool callback_safe()
{
  //judge sensors status
  static int gps_flag = 0;

  if (gps_flag != 2)
  {
    ROS_ERROR("Warning! Please check your GPS sensors!!!!");
  }
}
void icvCarlaControlNode::Noderun()
{
  ROS_INFO_STREAM("icv_carla_control  start");
  ros::Rate loop_rate(LOOP_RATE_);
  double angle = 0; //current angle
  double out_throttle = 0;
  double out_brake = 0;

  while (ros::ok())
  {
    if (callback_imu_flag == 0)
      ROS_WARN("gps not receive !!!!");

#ifdef __DEBUG__SURPPORT_
    if (callback_imu_flag == 1)
    {
      // Path_generator(_R_test,_delta_d,Yaw_carla);//计算测试路径  //实车屏蔽
      // data_file_input(); //提取txt路径测试数据
    }
#endif

    if (callback_imu_flag == 1)
    {
      VehicelStatusSend();
      zz.run_follower(&angle);
      if (VehicleMode_flag == 1 || VehicleMode_flag == 2)
      {
        /**carla中使用**/
        target_speedcarla = speed_plan_desir * 3.6;
        zz.run_speedcontrol(&out_throttle, &out_brake, target_speedcarla);
        angle = -angle; //carla left - right +//autoware zuo+ you-
        ROS_INFO("Carla Vehicle SteerSend=  %lf", angle);
        publishVehicleinstruction(angle, out_throttle, out_brake); //carla  pub   //实车屏蔽
      }
      if (VehicleMode_flag == 3)
      {
        /***小鹏实车测试使用***/
        angle = angle * (180 / PI) * 15.7; //xp left +  right -
        ROS_INFO("XpMotors Vehicle SteerSend=  %lf", angle);
        ROS_INFO("XpMotors Vehicle target_speedxp=  %lf", speed_plan_desir);
        CarconstructionPub(angle, speed_plan_desir); //发布小鹏的控制信息
      }
    }
    // for visualization with Rviz
    pubVechilepose2rviz(XpPoint_rec);

    ros::spinOnce();
    loop_rate.sleep();
  }
}
void icvCarlaControlNode::pubVechilepose2rviz(VehicleStatus carpose)
{
  geometry_msgs::Pose carpoint, pre_point; //车辆当前姿态
  carpoint.position.x = XpPoint_rec.x;
  carpoint.position.y = XpPoint_rec.y;
  geometry_msgs::Quaternion quaternion; //定义四元数
  double YY = XpPoint_rec.theta / 180 * PI;
  quaternion = tf::createQuaternionMsgFromRollPitchYaw(0, 0, YY); //欧拉角
  carpoint.orientation = quaternion;
  visualization_msgs::Marker Vehicle_msg = pubVecilePosetoRviz(carpoint);
  pub_vehi_pose.publish(Vehicle_msg);

  double x_pre, y_pre;
  float d_error;
  zz.sendXYpre(&x_pre, &y_pre, &d_error);
  pre_point.position.x = x_pre;
  pre_point.position.y = y_pre;

  /*计算误差平均值和方差*/

  if (resultSet.size() < 1000)
  {
    resultSet.push_back(abs(d_error));
  }
  else
  {
    resultSet.clear();
  }
  double sum = std::accumulate(std::begin(resultSet), std::end(resultSet), 0.0);
  double mean = sum / resultSet.size(); //均值
  double accum = 0.0;
  std::for_each(std::begin(resultSet), std::end(resultSet), [&](const double d) {
    accum += (d - mean) * (d - mean);
  });
  double stdev = sqrt(accum / (resultSet.size() - 1)); //方差

  geometry_msgs::Vector3 error;
  error.x = d_error;
  error.y = mean;
  error.z = stdev;
  visualization_msgs::Marker pre_point_msg = pubPrepointtoRviz(pre_point);

  pub_follow_error.publish(error);
  pub_pre_point.publish(pre_point_msg);
}
void icvCarlaControlNode::CarconstructionPub(double steer_send, double SpeedReq)
{

  xpmotors_can_msgs::AutoCtlReq ctrmsg;
  ctrmsg.EPSAngleReq = steer_send;

  if (!CurDriveMode)
  {
    ROS_INFO("Vehicle is Humanmode!!!");
    ctrmsg.EPSAngleReq = 0;
    ctrmsg.AutoMode = 1;
    ctrmsg.TarSpeedReq = 1;
    usleep(1000);
  }
  else if (EPBState)
  {
    ROS_DEBUG("EPB is hold!!!");
    for (int i = 0; i < 30; i++)
    {
      ctrmsg.AutoMode = 1;
      ctrmsg.TarSpeedReq = 1;
      usleep(50000);
      pub_xp_.publish(ctrmsg);
    }
    for (int i = 0; i < 20; i++)
    {
      ROS_DEBUG("EPB is hold!!!");
      ctrmsg.AutoMode = 1;
      ctrmsg.TarSpeedReq = 0;
      usleep(1000);
      pub_xp_.publish(ctrmsg);
    }
  }
  else
  {
    ROS_DEBUG("Vehicle is Automode!!!");
    ctrmsg.AutoMode = 1;
    ctrmsg.TarSpeedReq = SpeedReq; //m/s
  }

  pub_xp_.publish(ctrmsg);
}

void icvCarlaControlNode::VehicelStatusSend()
{
  //update vehicle coordinate(x(m),y(m),yaw(deg)),chassis data speed(m/s),steerangle(deg)
  zz.SetVehicleStatus(XpPoint_rec.x, XpPoint_rec.y, XpPoint_rec.theta, XpPoint_rec.speed_rec, XpPoint_rec.steer_rec);
}

/******************************************************************************************************carla*********************************************************************************************************************************/
#ifdef __DEBUG__SURPPORT_
/*仿真路径生成,生成固定转弯半径的路径点，距离长度500m,左手坐标系拟合路径*/
void icvCarlaControlNode::Path_generator(double R, double delta_d, double headingangle)
{
  testPath.poses.clear();
  geometry_msgs::PoseStamped Point;
  double delta = 0;
  double delta_run = 0;
  double delta_max = PI / 2; //转向角度
  double x = XpPoint_rec.x;
  double y = XpPoint_rec.y;
  double s = 0;          //实时计算的路径长度
  double s1 = 0, s2 = 0; //与弯道连接的直道
  double s_max = 500;    //限制路径长度
  if (R < 100)
    delta_max = PI;
  if (R < 1000)
  {
    //50m straight line
    while (s1 < 30)
    {
      x = x + delta_d * cos(headingangle / 180 * PI);
      y = y + delta_d * sin(headingangle / 180 * PI);
      Point.pose.position.x = x;
      Point.pose.position.y = y;
      s1 += delta_d;
      testPath.poses.push_back(Point);
    }
    cout << "turn point X1,Y1=  " << x << " " << y << endl;
    //turn line
    delta = asin(delta_d / (2 * R));
    while (delta_run < delta_max)
    {
      x = x + delta_d * cos(delta_run + headingangle / 180 * PI);
      y = y + delta_d * sin(delta_run + headingangle / 180 * PI);
      delta_run += delta;
      s += delta_d;
      Point.pose.position.x = x;
      Point.pose.position.y = y;
      testPath.poses.push_back(Point);
      // cout << "delta_run " << delta_run << endl;
    }
    cout << "turn point X2,Y2=  " << x << " " << y << endl;
    //50m straight line
    while (s2 < 30)
    {
      x = x + delta_d * cos(delta_run + headingangle / 180 * PI);
      y = y + delta_d * sin(delta_run + headingangle / 180 * PI);
      Point.pose.position.x = x;
      Point.pose.position.y = y;
      s2 += delta_d;
      testPath.poses.push_back(Point);
    }
  }
  if (R > 5000)
  {
    while (s < s_max)
    {
      x = x + delta_d * cos(headingangle / 180 * PI);
      y = y + delta_d * sin(headingangle / 180 * PI);
      Point.pose.position.x = x;
      Point.pose.position.y = y;
      s += delta_d;
      testPath.poses.push_back(Point);
    }
  }
  testPath.header.frame_id = "map";
  pub_path.publish(testPath);
  zz.setWaypoints(testPath);
}
#endif

/**carlas仿真环境接口**/
void icvCarlaControlNode::VehicleStatusCallback(const carla_msgs::CarlaEgoVehicleStatus &msg) //实车屏蔽
{
  XpPoint_rec.speed_rec = msg.velocity;
  XpPoint_rec.steer_rec = msg.control.steer * 180 / PI;
  ROS_INFO_ONCE("I hear Carla Vehiclestatus!");
}
void icvCarlaControlNode::callbackFromCurrentPose(const geometry_msgs::PoseStampedConstPtr &msg) //实车屏蔽
{

  //carla左手坐标系
  XpPoint_rec.x = msg->pose.position.x;
  XpPoint_rec.y = msg->pose.position.y;
  Yaw_carla = qua_to_rpy(msg->pose.orientation);
  XpPoint_rec.theta = Yaw_carla;

  callback_imu_flag = true;
  // ROS_DEBUG("I hear callbackFromCurrentposr!");
}
void icvCarlaControlNode::WaypointCallback(const nav_msgs::Path &msg)
{
  //TODO::receive Path and sent it to Control
  zz.setWaypoints(msg);
  pub_path.publish(msg);
  ROS_DEBUG("I hear %ld Waypoints!", msg.poses.size());
}
void icvCarlaControlNode::publishVehicleinstruction(double steer, double throttle, bool brakeflag)
{
  //TODO::send control instruction to carla
  //input::steer(rad)  throttle(0~1) brakeflag(0,1)
  carla_msgs::CarlaEgoVehicleControl msg_pub1;
  msg_pub1.steer = steer;
  msg_pub1.throttle = throttle;
  msg_pub1.brake = brakeflag;
  pub1_.publish(msg_pub1);
}

/***********************************************************************************************小鹏车使用***********************************************************************************************/
/*读取轨迹点信息*/
#ifdef __ZZZ_SURPPORT__
void icvCarlaControlNode::callback_Path(const zzz_planning_msgs::DecisionTrajectory &msg)
{
  zz.setWaypoints(msg.trajectory); //轨迹获取
  speed_plan_desir = msg.desired_speed;
  callback_Path_flag = true;
  ROS_DEBUG("I hear pointsize %ld",msg.trajectory.poses.size());
}
#endif
#ifdef __DEBUG__SURPPORT_
void icvCarlaControlNode::data_file_input()
{
  nav_msgs::Path Path_file;
  Path_file.poses.clear();
  geometry_msgs::PoseStamped Point_file;

  //读取采集到的路径点信息
  ifstream infile1;
  infile1.open("/home/zx/Desktop/heqingpath.txt"); //打开文件
  if (!infile1.good())
  {
    ROS_ERROR("no Path file !!!please check your pathfile!!!!!");
  }
  char c;
  int lineCnt = 0;
  while (infile1.get(c))
  {
    if (c == '\n')
      lineCnt++;
  }
  infile1.close();
  infile1.open("/home/zx/Desktop/heqingpath.txt"); //打开文件
  for (int i = 0; i < lineCnt; i++)                //定义行循环
  {
    for (int j = 0; j < 2; j++) //定义列循环
    {
      switch (j)
      {
      case 0:
        infile1 >> Point_file.pose.position.x;
        break;
      case 1:
        infile1 >> Point_file.pose.position.y;
        break;
      default:
        break;
      }
    }
    Path_file.header.frame_id = "map";
    Path_file.header.stamp = ros::Time();
    Path_file.poses.push_back(Point_file);
  }
  infile1.close();
  pub_path.publish(Path_file);
  zz.setWaypoints(Path_file);
}
#endif

#ifdef __ZZZ_SURPPORT__
void icvCarlaControlNode::callback_egopose(const zzz_driver_msgs::RigidBodyStateStamped &msg)
{
  XpPoint_rec.x = msg.state.pose.pose.position.x;
  XpPoint_rec.y = msg.state.pose.pose.position.y;
  double Yaw = qua_to_rpy(msg.state.pose.pose.orientation);
  // Yaw += 90;
  // if (Yaw < 0)
  // {
  //   Yaw += 360;
  // }
  XpPoint_rec.theta = Yaw; //参数传递

  callback_imu_flag = true;
}
#endif

void icvCarlaControlNode::callback_auto_state_ex(const xpmotors_can_msgs::AutoStateEx &msg)
{
  //TODO::Vehicle Drive mode feedback

  StateTurningLight = msg.StateTurningLight;
  CurDriveMode = msg.CurDriveMode;
  StateBraking = msg.StateBraking;
  callback_auto_state_ex_flag = true;
  ROS_INFO_ONCE("I hear /xp/auto_state_ex topic ~~~~~~");
}

void icvCarlaControlNode::callback_auto_state(const xpmotors_can_msgs::AutoState &msg)
{
  //TODO::Vehicle Chassis data

  EPBState = msg.EPBState;
  GearState = msg.GearState;
  BrkPedal = msg.BrkPedal;
  AccPedal = msg.AccPedal;
  callback_auto_state_flag = true;
  ROS_INFO_ONCE("I hear /auto/state topic ~~~~~~");
}

void icvCarlaControlNode::callback_esc(const xpmotors_can_msgs::ESCStatus &msg)
{
  //TODO::Vehicle Drive speedfeedback (km/h)
  RRWheelSpd = msg.RRWheelSpd;
  LFWheelSpd = msg.LFWheelSpd;
  RFWheelSpd = msg.RFWheelSpd;
  LRWheelSpd = msg.LRWheelSpd;
  callback_esc_flag = true;
  XpPoint_rec.speed_rec = RRWheelSpd / 3.6; //m/s
  ROS_INFO_ONCE("I hear /xp/esc topic ~~~~~~");
}

void icvCarlaControlNode::callback_eps(const xpmotors_can_msgs::EPSStatus &msg)
{
  //TODO::Vehicle EPS data feedback

  AngleSpd = msg.AngleSpd;
  Angle = msg.Angle;
  StrngWhlTorq = msg.StrngWhlTorq;
  XpPoint_rec.steer_rec = msg.Angle; //zx    度数
  callback_eps_flag = true;
  ROS_INFO_ONCE("I hear /xp/eps topic ~~~~~~");
}

void icvCarlaControlNode::callback_gpsvel(const geometry_msgs::TwistWithCovarianceStamped &msg)
{
  //  state.state.twist.twist.linear.x=msg.twist.twist.linear.x;
  //  state.state.twist.twist.linear.y=msg.twist.twist.linear.y;
  //  state.state.twist.twist.linear.z=0;
  ROS_INFO_ONCE("I hear /gps/vel topic ~~~~~~");
}

/*IMU航向角*/
void icvCarlaControlNode::callback_imu(const sensor_msgs::Imu &msg)
{
  double Yaw = 0;

  Imu_angle_acc_x = msg.angular_velocity.x;
  Imu_angle_acc_y = msg.angular_velocity.y;
  Imu_angle_acc_z = msg.angular_velocity.z;

  Imu_linear_accX = msg.linear_acceleration.x;
  Imu_linear_accY = msg.linear_acceleration.y;
  Imu_linear_accZ = msg.linear_acceleration.z;

  Yaw = qua_to_rpy(msg.orientation);

  Yaw += 90;
  if (Yaw < 0)
  {
    Yaw += 360;
  }
  XpPoint_rec.theta = Yaw; //参数传递
  ROS_INFO_ONCE("I hear /imu/data topic ~~~~~~");
  callback_imu_flag = true;
}

//receive UTM coordinate from localization
void icvCarlaControlNode::callback_gpsodom(const nav_msgs::Odometry &msg)
{
  float glo_x, glo_y, glo_yaw;
  glo_x = msg.pose.pose.position.x;
  glo_y = msg.pose.pose.position.y;
  glo_yaw = qua_to_rpy(msg.pose.pose.orientation);
  XpPoint_rec.theta = glo_yaw;
  XpPoint_rec.x = glo_x;
  XpPoint_rec.y = glo_y;
  // cout << "X , Y =  " << glo_x << " " << glo_y << " " << glo_yaw << endl;
  callack_odom_flag = true;
  ROS_INFO_ONCE("I hear /gps/odom topic ~~~~~~");
}
/*GPS转xy*/
void icvCarlaControlNode::callback_gpsfix(const sensor_msgs::NavSatFix &msg)
{

  callback_gps_flag = true;

  geographic_msgs::GeoPointStampedPtr gps_msg(new geographic_msgs::GeoPointStamped());
  gps_msg->position.latitude = msg.latitude;
  gps_msg->position.longitude = msg.longitude;
  gps_msg->position.altitude = msg.altitude;

  geodesy::UTMPoint utm;
  geodesy::fromMsg(gps_msg->position, utm);
  Eigen::Vector3d xyz(utm.easting, utm.northing, utm.altitude);

  if (VehicleMode_flag == 3)
  {
    XpPoint_rec.x = utm.easting - 442867;
    XpPoint_rec.y = utm.northing - 4427888;
  }

  callback_gpsfix_flag = true;
  ROS_INFO_ONCE("I hear /gps/fix topic ~~~~~~");
  double yawstate = atan2(yy - yyold, xx - xxold) / PI * 180;
  if (yawstate < 0)
    yawstate += 360;

  // ofstream out;
  // out.open("/home/zx/follow_carla/Yawdata0414.txt",std::ios::out | std::ios::app);
  // out<<setiosflags(ios::fixed)<<setprecision(3)<<yawstate<<" "<<XpPoint_rec.theta<<" "<<xx<<" "<<yy<<" "<<yy-yyold<<" "<<xx-xxold<<" "<<yyold<<" "<<xxold<<endl;
  // out.close();
  xxold = xx;
  yyold = yy;
}
/*四元素转航向角*/
double icvCarlaControlNode::qua_to_rpy(geometry_msgs::Quaternion posedata)
{
  float w = posedata.w;
  float x = posedata.x;
  float y = posedata.y;
  float z = posedata.z;

  float R = atan2((2 * (w * x + y * z)), (1 - 2 * (x * x + y * y)));
  float P = asin(2 * (w * y - z * x));
  float Y = atan2((2 * (w * z + x * y)), (1 - 2 * (z * z + y * y)));
  Y = Y * 180 / 3.141592654;

  return Y;
}
//RPY转四元素//
geometry_msgs::Quaternion icvCarlaControlNode::rpy_to_qua(double Yaw, double Pitch, double Roll)
{

  geometry_msgs::Quaternion qua;
  Yaw = Yaw * PI / 180;
  Pitch = 0 * PI / 180;
  Roll = 0 * PI / 180;

  double cy = cos(Yaw * 0.5);
  double sy = sin(Yaw * 0.5);
  double cp = cos(Pitch * 0.5);
  double sp = sin(Pitch * 0.5);
  double cr = cos(Roll * 0.5);
  double sr = sin(Roll * 0.5);

  qua.w = cy * cp * cr + sy * sp * sr;
  qua.x = cy * cp * sr - sy * sp * cr;
  qua.y = sy * cp * sr + cy * sp * cr;
  qua.z = sy * cp * cr - cy * sp * sr;

  return qua;
}

} // namespace icv
