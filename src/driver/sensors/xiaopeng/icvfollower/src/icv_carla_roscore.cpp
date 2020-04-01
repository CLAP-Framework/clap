
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
  ros::param::get("~target_speedcarla", target_speedcarla);
  // ros::param::get("~target_speedxp",target_speedxp);
  ros::param::get("~carla_flag", carla_flag);
  ros::param::get("~_R_test", _R_test);
  ros::param::get("~_delta_d", _delta_d);
  target_speedcarla = target_speedcarla * 3.6;
  if (carla_flag == 1)
  {
    /**carla仿真使用**/
    // setup subscriber
    // sub1_ = nh_.subscribe("/carla/ego_vehicle/waypoints", 10, &icvCarlaControlNode::WaypointCallback, this);//实车屏蔽
    // sub2_ = nh_.subscribe("/gnss_pose", 10, &icvCarlaControlNode::callbackFromCurrentPose, this);//实车屏蔽
    // sub3_ = nh_.subscribe("/carla/ego_vehicle/vehicle_status", 10, &icvCarlaControlNode::VehicleStatusCallback, this);  //实车屏蔽
    // // setup publisher
    // pub1_ = nh_.advertise<carla_msgs::CarlaEgoVehicleControl>("/carla/ego_vehicle/vehicle_control_cmd", 10);//实车屏蔽
  }
  else
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
    sub_zzz_path_ = nh_.subscribe("/zzz/planning/decision_trajectory", 10, &icvCarlaControlNode::callback_Path, this);

    pub_xp_ = nh_.advertise<xpmotors_can_msgs::AutoCtlReq>("/xp/auto_control", 1000);
  }
}


void icvCarlaControlNode::Noderun()
{
  ROS_INFO_STREAM("icv_carla_control  start");
  ros::Rate loop_rate(LOOP_RATE_);
  int cou = 0;
  ofstream out;
  double angle = 0;     //前轮转角
  double angle_old = 0; //上一时刻角度
  double out_throttle = 0;
  double out_brake = 0;
  double factor = 0;
  double v_cur = 0; //carla 使用当前车速

  while (ros::ok())
  {
    if (callback_imu_flag == 1 & cou < 10)
    {
      // Path_generator(_R_test,_delta_d,Yaw_carla);//计算测试路径  //实车屏蔽
      // data_file_input();//提取txt路径测试数据
      cou += 1;
    }
    if (callback_imu_flag == 1 & cou > 9)
    {
      VehicelStatusSend();
      zz.run_follower(&angle);
      if (carla_flag)
      {
        /**carla中使用**/
        zz.run_speedcontrol(&out_throttle, &out_brake, target_speedcarla);
        angle = -angle; //carla left - right +
        ROS_DEBUG("Carla Vehicle SteerSend=  %lf", angle);
        // cout<<"   "<<out_throttle<<"     223233  "<<out_brake<<endl;
        // publishVehicleinstruction(angle,out_throttle,out_brake); //carla  pub   //实车屏蔽
      }
      else
      {
        /***小鹏实车测试使用***/
        angle = angle * (180 / PI) * 15.7; //xp left +  right -
        ROS_INFO("XpMotors Vehicle SteerSend=  %lf", angle);
        ROS_INFO("XpMotors Vehicle target_speedxp=  %lf", target_speedxp);
        CarconstructionPub(angle, target_speedxp); //发布小鹏的控制信息
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
  zz.sendXYpre(&x_pre, &y_pre,&d_error);
  pre_point.position.x = x_pre;
  pre_point.position.y = y_pre;

  if(resultSet.size()<10000)
  {
    resultSet.push_back(abs(d_error)) ;  
  }
  else
  {
    resultSet.clear();
  }
  double sum = std::accumulate(std::begin(resultSet), std::end(resultSet), 0.0);  
  double mean =  sum / resultSet.size(); //均值    
  double accum  = 0.0;  
  std::for_each (std::begin(resultSet), std::end(resultSet), [&](const double d) 
  {  
  accum  += (d-mean)*(d-mean);  
  });    
  double stdev = sqrt(accum/(resultSet.size()-1)); //方差 

  geometry_msgs::Vector3 error;
  error.x=d_error;
  error.y=mean;
  error.z=stdev;
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
    // cout<<"CurDriveMode "<<CurDriveMode<<endl;
    ROS_INFO("Vehicle is Humanmode!!!");
    ctrmsg.EPSAngleReq = 0;
    ctrmsg.AutoMode = 1;
    ctrmsg.TarSpeedReq = 1;
    usleep(1000);
  }
  else if (EPBState)
  {
    // cout<<"EPBState   "<<EPBState<<endl;
    for (int i = 0; i < 40; i++)
    {
      ROS_INFO("EPB is hold!!!");
      ctrmsg.AutoMode = 1;
      ctrmsg.TarSpeedReq = 1;
      usleep(50000);
      pub_xp_.publish(ctrmsg);
    }
    for (int i = 0; i < 20; i++)
    {
      ROS_INFO("EPB is hold!!!");
      ctrmsg.AutoMode = 1;
      ctrmsg.TarSpeedReq = 0;
      usleep(5000);
      pub_xp_.publish(ctrmsg);
    }
  }
  else
  {
    ROS_INFO("Vehicle is Automode!!!");
    ctrmsg.AutoMode = 1;
    ctrmsg.TarSpeedReq = SpeedReq; //m/s
  }
  cout<<" ctrmsg.TarSpeedReq "<<ctrmsg.TarSpeedReq<<endl;//zx
  cout<<" ctrmsg.EPSAngleReq "<<ctrmsg.EPSAngleReq<<endl;//zx
  pub_xp_.publish(ctrmsg);
}
//汇总整个类接口数据然后传递，车辆当前状态，定位，路径信息
void icvCarlaControlNode::VehicelStatusSend()
{
  zz.SetVehicleStatus(XpPoint_rec.x, XpPoint_rec.y, XpPoint_rec.theta, XpPoint_rec.speed_rec, XpPoint_rec.steer_rec);
}
/******************************************************************************************************carla*********************************************************************************************************************************/
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
  double s = 0;       //实时计算的路径长度
  double s_max = 500; //限制路径长度
  if (R < 100)
    delta_max = PI;
  if (R < 1000)
  {
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
      cout << "delta_run " << delta_run << endl;
    }
  }
  if (R > 5000)
  {
    while (s < s_max)
    {
      x = x + delta_d * sin(headingangle / 180 * PI);
      y = y + delta_d * cos(headingangle / 180 * PI);
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

/**carlas仿真环境接口**/
// void icvCarlaControlNode::VehicleStatusCallback(const carla_msgs::CarlaEgoVehicleStatus &msg)//实车屏蔽
// {
//   XpPoint_rec.speed_rec=msg.velocity;
//   XpPoint_rec.steer_rec=msg.control.steer*180/PI;
//   // ROS_INFO("I hear Vehiclestatus!");
// }
// void icvCarlaControlNode::callbackFromCurrentPose(const geometry_msgs::PoseStampedConstPtr &msg)//实车屏蔽
// {

//     //carla左手坐标系
//     XpPoint_rec.x=msg->pose.position.x;
//     XpPoint_rec.y=msg->pose.position.y;
//     Yaw_carla=qua_to_rpy(msg->pose.orientation);
//     XpPoint_rec.theta=Yaw_carla;

//     callback_imu_flag=true;
//     // ROS_INFO("I hear callbackFromCurrentposr!");
// }
// void icvCarlaControlNode::WaypointCallback(const nav_msgs::Path &msg)//实车屏蔽
// {
//   zz.setWaypoints(msg);
//   pub_path.publish(msg);
//   ROS_INFO("I hear %d Waypoints!",msg.poses.size());
// }
// void icvCarlaControlNode::publishVehicleinstruction(double steer,double throttle,bool brakeflag)//方向盘、油门值、制动指令//实车屏蔽
// {
//     carla_msgs::CarlaEgoVehicleControl msg_pub1;
//     msg_pub1.steer=steer;
//     msg_pub1.throttle=throttle;
//     msg_pub1.brake=brakeflag;
//     pub1_.publish(msg_pub1);

// }
/***********************************************************************************************小鹏车使用***********************************************************************************************/
/*读取轨迹点信息*/
void icvCarlaControlNode::callback_Path(const zzz_planning_msgs::DecisionTrajectory &msg)
{
  zz.setWaypoints(msg.trajectory);    //轨迹获取
  target_speedxp = msg.desired_speed; //速度获取
  callback_Path_flag = true;
}
void icvCarlaControlNode::data_file_input()
{
  nav_msgs::Path Path_file;
  Path_file.poses.clear();
  geometry_msgs::PoseStamped Point_file;

  //读取采集到的路径点信息
  ifstream infile1;
  infile1.open("/home/icv/follow_carla/wPath.txt"); //打开文件
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
  infile1.open("/home/icv/follow_carla/wPath.txt"); //打开文件
  for (int i = 0; i < lineCnt; i++)                 //定义行循环
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

void icvCarlaControlNode::callback_egopose(const zzz_driver_msgs::RigidBodyStateStamped &msg)
{
  XpPoint_rec.x= msg.state.pose.pose.position.x;
  XpPoint_rec.y= msg.state.pose.pose.position.y;
  double Yaw = qua_to_rpy(msg.state.pose.pose.orientation);
  // Yaw += 90;
  // if (Yaw < 0)
  // {
  //   Yaw += 360;
  // }
  XpPoint_rec.theta = Yaw; //参数传递

  callback_imu_flag = true;
}
/**车辆驾驶模式反馈*/
void icvCarlaControlNode::callback_auto_state_ex(const xpmotors_can_msgs::AutoStateEx &msg)
{
  StateTurningLight = msg.StateTurningLight;
  CurDriveMode = msg.CurDriveMode;
  StateBraking = msg.StateBraking;
  callback_auto_state_ex_flag = true;
}
/**车辆底盘反馈*/
void icvCarlaControlNode::callback_auto_state(const xpmotors_can_msgs::AutoState &msg)
{
  EPBState = msg.EPBState;
  GearState = msg.GearState;
  BrkPedal = msg.BrkPedal;
  AccPedal = msg.AccPedal;
  callback_auto_state_flag = true;
}
/*车辆rsc反馈*/
void icvCarlaControlNode::callback_esc(const xpmotors_can_msgs::ESCStatus &msg)
{
  RRWheelSpd = msg.RRWheelSpd;
  LFWheelSpd = msg.LFWheelSpd;
  RFWheelSpd = msg.RFWheelSpd;
  LRWheelSpd = msg.LRWheelSpd;
  callback_esc_flag = true;
  XpPoint_rec.speed_rec = 0.01 * RRWheelSpd / 3.6; //m/s
}
/*车辆EPS反馈*/
void icvCarlaControlNode::callback_eps(const xpmotors_can_msgs::EPSStatus &msg)
{

  AngleSpd = msg.AngleSpd;
  Angle = msg.Angle;
  StrngWhlTorq = msg.StrngWhlTorq;
  XpPoint_rec.steer_rec = msg.Angle * 0.02; //zx    度数
  callback_eps_flag = true;
}
/*GPS速度*/
// void icvCarlaControlNode::callback_gpsvel(const geometry_msgs::TwistWithCovarianceStamped &msg)
// {
//   //  state.state.twist.twist.linear.x=msg.twist.twist.linear.x;
//   //  state.state.twist.twist.linear.y=msg.twist.twist.linear.y;
//   //  state.state.twist.twist.linear.z=0;
// }

// /*IMU航向角*/
// void icvCarlaControlNode::callback_imu(const sensor_msgs::Imu &msg)
// {
//   double Yaw = 0;
//   double Imu_accX = 0;
//   double Imu_accY = 0;
//   double Imu_accZ = 0;
//   Imu_accX = msg.linear_acceleration.x;
//   Imu_accY = msg.linear_acceleration.y;
//   Imu_accZ = msg.linear_acceleration.z;

//   Yaw = qua_to_rpy(msg.orientation);

//   Yaw += 90;
//   if (Yaw < 0)
//   {
//     Yaw += 360;
//   }
//   XpPoint_rec.theta = Yaw; //参数传递

//   callback_imu_flag = true;
// }
// /*GPS转xy*/
// void icvCarlaControlNode::callback_gpsfix(const sensor_msgs::NavSatFix &msg)
// {

//   callback_gps_flag = true;

//   geographic_msgs::GeoPointStampedPtr gps_msg(new geographic_msgs::GeoPointStamped());
//   gps_msg->position.latitude = msg.latitude;
//   gps_msg->position.longitude = msg.longitude;
//   gps_msg->position.altitude = msg.altitude;

//   geodesy::UTMPoint utm;
//   geodesy::fromMsg(gps_msg->position, utm);
//   Eigen::Vector3d xyz(utm.easting, utm.northing, utm.altitude);

//   XpPoint_rec.x = utm.easting - 442867;
//   XpPoint_rec.y = utm.northing - 4427888;

//   callback_gpsfix_flag = true;
// }
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
} // namespace icv