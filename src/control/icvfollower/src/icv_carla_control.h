
#ifndef ICV_CARLA_CONTROL_H
#define ICV_CARLA_CONTROL_H

#define __CARLA_SURPPORT__ 1
#define __ZZZ_SURPPORT__ 1
#define __DEBUG__SURPPORT_ 1

#include <stdio.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <cmath>
#include <fstream>
#include <iostream>
#include <numeric>
// #include "waypoint_follower/libwaypoint_follower.h"
#include <math.h>
#include <tf/tf.h>

#include <map>
#include <thread>
#include <vector>

// inlcude iostream and string libraries
#include <time.h>

#include <Eigen/Core>
#include <Eigen/QR>
#include <string>
#include <vector>

#define __CARLA_SURPPORT__ 1

#ifdef __CARLA_SURPPORT__
#include "carla_msgs/CarlaEgoVehicleControl.h"
#include "carla_msgs/CarlaEgoVehicleStatus.h"
#endif

#include <geodesy/utm.h>
#include <geodesy/wgs84.h>
#include <geographic_msgs/GeoPointStamped.h>
#include <yaml-cpp/yaml.h>

#include "nav_msgs/Path.h"

// include eigen
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/QR>

// autoware
#if 0

#include "autoware_config_msgs/ConfigWaypointFollower.h"
#include "autoware_msgs/ControlCommandStamped.h"
#include "autoware_msgs/Lane.h"

#endif

#include <xpmotors_can_msgs/AutoCtlReq.h>

#include "xpmotors_can_msgs/AutoState.h"
#include "xpmotors_can_msgs/AutoStateEx.h"
#include "xpmotors_can_msgs/EPSStatus.h"
#include "xpmotors_can_msgs/ESCStatus.h"

#ifdef __ZZZ_SURPPORT__
#include "zzz_driver_msgs/RigidBodyStateStamped.h"
#include "zzz_planning_msgs/DecisionTrajectory.h"
#endif

#include "icv_carla_rviz.h"

using namespace Eigen;
using namespace std;

namespace icv {
class icvCarlaControl {
 private:
  /* data */
  float X, Y, theta;
  float v = 0;  //车辆速度m/s
  // vehicle mode param
  float lf = 1.5;  //
  float lr = 1.5;  //
  float L = lr + lf;
  float m = 1818.2;        // mass of Vehicle
  float Ccf = 66900;       //前轮侧偏刚度
  float Ccr = 62700;       //后轮侧偏刚度
  float Clf = 66900;       //前轮纵向刚度
  float Clr = 62700;       //后轮纵向刚度
  float delta_f = 0;       //前轮偏角度
  float delta_f_old = 0;   //前轮偏角度
  float Sf = 0.2;          //前轮纵向滑移率
  float Sr = 0.2;          //后轮纵向滑移率
  float I = 4175;          //车辆质心转动惯量
  float wheel_base = 2.7;  //车辆轴距
  float g = 9.8;
  float R_min = 5.13;  //最小转向半径
  float K = 15.7;      //方向盘传动比
  /*vehicle state constrate */
  float r = 0;  //横摆角速度
  float x_dot = 0.000001, y_dot, phi_dot, phi;
  float dt = 0.05;
  float Acc_max = 3;   // the max limit ACC
  float jerk_max = 5;  // jerk_max加速度变化率上限

  // yan
  int _flag = 0;
  double _wheel_angle_degree_last = 0;
  double _cal_wheel_angle = 0;
  int _record_cnt = 0;
  float _vehicle_wheel_angle;   // feedback steers
  float _keep_wheel_angle = 0;  // record last time of feedback steers
  float _delta_T = 0.05;
  vector<double> _path_x;
  vector<double> _path_y;
  // parameters of preview
  float _p1_post = 14.5;
  float _p2_post = 14.5;
  float _p3_post = 14.5;
  float _p4_post = 14.5;
  float _p5_post = 14.5;
  float _p6_post = 14.5;
  float _p7_post = 14.5;
  float _p8_post = 14.5;
  float _p9_post = 14.5;

  float _lat_gain = 0;
  float _lat_angle_gain = 0;
  float pre_K = 0;          // the weight of preview control
  float feedforward_K = 0;  //  Curvature feed forward gravity
                            //  （pre_K+feedforward_K=1）
  float feedforward_P = 0;  // yumiaowucha

  float __distance_pre = 0.7;
  float _dis_pre_max = 20;              // maxmum lookahead distance
  float _dis_pre_min = 2;               // minimum lookahead distance
  float _wheel_angle_degree_max = 470;  // limit maxmum steer angle
  float _wheel_angle_degree_sec = 200;  // limit max steerangle in one sec
  float _t_preview = 2.8;               // preview time 1
  float _t_preview_turn = 0;            // preview time 2
  int nextpoint_num = 0;                // nearest index of pathpoint
  float _kappa = 0;                     // Curvature compensation coefficient

  double PI = 3.141592654;
  int Waypoints_size = 0;

  /**subdata flag*/
  bool current_pose_flag = false;
  bool waypoints_flag = false;
  bool setVehiclestauts_flag = false;
  int _keep_steer = 0;

  // Yaml add parameters
  float _smoothsteernum = 0;
  float point_distancemax = 2;  //最大点间距,设置为param中参数调用   Yamlparam
  float car_path_dis = 20;       // Yamlparam
  int path_size_min = 0;         // pathsize mininuber
  float check_maxerrordis = 30;  //仿真模型中可以存在的最大误差,YAML
  double kappa_min = 1 / 9e10;  // Yamlparam
  float _d_0 = 1.957;           //起始距离.Yamlparam
  float _d_dis_min = 2.5;
  float _d_dis_max = 40;
  float _d_dis_min_k = 5;
  float _d_dis_max_k = 40;
  float _KK = 0;
  float _l1 = 0;
  float _l2 = 0;
  float _l3 = 0;
  float _l4 = 0;
  float _T = 0;

  /****数据采集param****/
  float ctedata[8];  // cte_d,steer,预瞄误差，x，y，曲率，系数，速度
  double sample_time = 0;  // one loop  time

  /**数据传递参数*/
  float target_speed = 0;  // target speed output
  float target_speed_limit = 0;

  /**PID参数定义****/
  double error_p = 0;       // kp_error
  double error_i = 0;       // ki_error
  double error_d = 0;       // kd_error
  double error_dp = 0;      // last time kp error、use to calculate ki_error
  double out_insturct = 0;  // output parameter of PIDcontrol
  /**/
  bool follower_flag = false;

  /***/
  double lookahead_distance_ = 0;
  double Beta = 0;

  /*smooth filter**/
  vector<double> filterdata;

  /**/
  float _x_preview_rviz;  // the x coordinates of preview point
  float _y_preview_rviz;  // the y coordinates of preview point

  /*结构体定义*/
  typedef struct Path {
    float x = 0;
    float y = 0;
    float z = 0;
    float theta = 0;  //度。
    float pitch = 0;
    float roll = 0;
    float v = 0;        //车辆速度m/s
    float x_dot = 0;    //车辆纵向速度
    float y_dot = 0;    //车辆横向速度
    float phi_dot = 0;  //车辆横向偏转角速度
    float steer_rec = 0;
  } Pathpoint;
  typedef struct PID {
    float Kp = 0;
    float Ki = 0;
    float Kd = 0;
  } pid_;
  PID _v_pid;            //速度的PID系数
  Pathpoint Pose_start;  // Vehicle 初始点
  Pathpoint Waypoints[10240];  //的矩阵，用于存放路径点，车速  定义结构体数组
  vector<Pathpoint> Waypointss;  //创建点的数组
  Pathpoint Waypoint;            //实例化路径

  Pathpoint Current_Point;                     //车辆实时坐标点
  geometry_msgs::Point next_target_position_;  //当前车辆坐标下一个点
  geometry_msgs::Pose current_pose_;           //车辆当前姿态
  carla_msgs::CarlaEgoVehicleStatus vehicle_status;  //车辆当前底盘状态
  nav_msgs::Path Path;                               //路径点信息

  void controlInit();
  double PIDcontrolbrake(double error, PID pid);
  double PIDcontrolacc(double error, PID pid);

  void Vehiclemodeupdate1(Pathpoint coordinate, float delta_f, float T,
                          float *x_vehicle, float *y_vehicle, float *heading);
  void Vehiclemodeupdate2(Pathpoint coordinate, float delta_f, float T,
                          float *x_vehicle, float *y_vehicle, float *heading);

  inline double deg2rad(double deg);
  // control funtions
  double ControlStr(double _vehicle_heading_theta, double _vehicle_v,
                    double _vehicle_x, double _vehicle_y);

  //跟踪方案二
  double ControlStr2(float _vehicle_x, float _vehicle_y,
                     float _vehicle_heading_theta, float _vehicle_v);

  float find_min_point(float _preview_x_rec, float _preview_y_rec,
                       int *count_min);
  float cal_transfer_val(float _d_dis_rec);
  // float limit_wheel_val(float _wheel_last_ref_rec,
  //                       float _wheel_angle_degree_rec);
  float limit_wheel_val(float _wheel_last_ref_rec,
                        float _wheel_angle_degree_rec);

  //多项式拟合
  Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                          int order);
  double polyeval(Eigen::VectorXd coeffs, double x);

  void pathdeal();  //计算路径偏差
  bool StateConstraint(float v, float angle, float T);
  float tiremodel(float beita, float cetia, float v, float r);
  double Velocitytopredict(
      float velocity, float distance_pre,
      float
          velocity_pre);  //小鹏速度模块，输入当前车速为km/h,预瞄距离，期望速度
  double limitParamChangeUnit(double param, double max_param,
                              double min_param);  //限制幅值函数
  double CalculateCur(geometry_msgs::Point P1, geometry_msgs::Point P2,
                      geometry_msgs::Point P3);  //三点计算曲率
  double smoothfilter(double param,
                      int smoothsize);  //多点平滑滤波器参数为平滑量，平滑点数
  geometry_msgs::Point calcRelativeCoordinate(
      geometry_msgs::Point point_msg,
      geometry_msgs::Pose current_pose);  //坐标转换
  double qua_to_rpy(geometry_msgs::Quaternion posedata);
  geometry_msgs::Quaternion rpy_to_qua(double Yaw, double Pitch, double Roll);
  void calcYaw(vector<double> pathx, vector<double> pathy, double *yaw);
  void selectCoefficient(double v_s, double kappa_s, float *G);  //选择系数

  bool Pathfollowcheck();  //路径检测处理
  bool PathSmoothCheck();  //求解path各曲率处,有问题返回bool判断标志
  bool Pathlength(
      double *
          pathlength);  //计算路径长度和每两个点的间距,判断是否正常,是否需要插值.

 public:
  icvCarlaControl(/* args */);
  ~icvCarlaControl();

  void run_follower(double *out_steerangle);
  void run_speedcontrol(double *out_throttle, double *out_brake, double tar_v);

  // debug data to txt file
  void data2file();

  //更新车辆坐标，车辆底盘反馈（车速，方向盘角度）
  void SetVehicleStatus(double _x, double _y, double _theta, double _speed,
                        double _vehicle_steerangle) {
    // TODO::update vehicle chassis feedback data
    // coordinate(x,y,theta)(m,m,deg),speed(m/s),steerangle_rec(deg)
    Current_Point.x = _x;
    Current_Point.y = _y;
    Current_Point.theta = _theta;
    Current_Point.v = _speed;
    _vehicle_wheel_angle = _vehicle_steerangle;
    current_pose_flag = true;
    setVehiclestauts_flag = true;
  }
  double angle_velocity_z = 0;
  void setImudata(const sensor_msgs::Imu &msg) {
    // todo::传递横摆角速度m/s????
    angle_velocity_z = msg.angular_velocity.z;
  }
  void setcarlaomodom(const nav_msgs::Odometry &msg) {
    angle_velocity_z = msg.twist.twist.angular.z;
    cout << angle_velocity_z << "  angle_velocity_z" << endl;
  }
  void setWaypoints(const nav_msgs::Path &msg_path) {
    // TODO::update Path
    // input::Path(x,y)
    _path_x.clear();
    _path_y.clear();
    for (int i = 0; i < msg_path.poses.size(); i++) {
      _path_x.push_back(msg_path.poses[i].pose.position.x);
      _path_y.push_back(msg_path.poses[i].pose.position.y);
      Waypoints[i].x = msg_path.poses[i].pose.position.x;
      Waypoints[i].y = msg_path.poses[i].pose.position.y;
    }
    Waypoints_size = _path_x.size();
    waypoints_flag = true;
  }
  //----------------------------------------------------------------
  void add_waypoint_path() {
    float dx = 0;
    float dy = 0;
    float final_pathpoint_yaw = 0;
    if (Waypoints_size > 2)
      final_pathpoint_yaw =
          atan2((_path_y[Waypoints_size - 1] - _path_y[Waypoints_size - 2]),
                (_path_x[Waypoints_size - 1] - _path_x[Waypoints_size - 2]));
    for (int i = 1; i < 100; i++) {
      dx = _path_x[Waypoints_size - 1] + 0.02 * i * cos(final_pathpoint_yaw);
      dy = _path_y[Waypoints_size - 1] + 0.02 * i * sin(final_pathpoint_yaw);
      _path_x.push_back(dx);
      _path_y.push_back(dy);
    }
  }
  /*out preview point show in rviz*/
  void sendXYpre(double *x, double *y, float *error) {
    // TODO::update Path
    // input::Path(x,y)
    *x = _x_preview_rviz;
    *y = _y_preview_rviz;
    *error = ctedata[0];
  }
  void safestatuscheck(bool sensor_flag, bool danger_flag) {
    // if(sensor_flag = true && danger_flag = false)
    // {
    //     _keep_steer = 1;
    // }
  }
};

}  // namespace icv

#endif  // ICV_CARLA_CONTROL_H
