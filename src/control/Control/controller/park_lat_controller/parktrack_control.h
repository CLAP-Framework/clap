
#ifndef PARKTRACK_CONTROL_H
#define PARKTRACK_CONTROL_H

#define __CARLA_SURPPORT__ 1

#define __DEBUG__SURPPORT__ 1

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
#include <geodesy/utm.h>
#include <geodesy/wgs84.h>
#include <geographic_msgs/GeoPointStamped.h>
#include <time.h>
#include <yaml-cpp/yaml.h>

#include <string>

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

// main
#include "../message/control_msgs.h"
using namespace Eigen;
using namespace std;

namespace Control {
class Parktrack_Control {
 private:
  /* data */
  float X, Y, theta;
  float v = 0;  //车辆速度m/s
  // vehicle mode param
  float lf = 1.255;  //
  float lr = 1.535;  //
  float L = lr + lf;
  float m = 1705;      // mass of Vehicle
  float Ccf = 116000;  //前轮侧偏刚度
  float Ccr = 187000;  //后轮侧偏刚度
  float Clf = 116000;  //前轮纵向刚度
  float Clr = 187000;  //后轮纵向刚度
  float delta_f = 0;   //前轮偏角度
  float delta_f_old = 0;
  float Sf = 0.2;          //前轮纵向滑移率
  float Sr = 0.2;          //后轮纵向滑移率
  float I = 4175;          //车辆质心转动惯量
  float wheel_base = 2.7;  //车辆轴距
  float g = 9.8;
  float R_min = 5.13;  //最小转向半径callback_imu_flag
  float K = 15.7;      //方向盘传动比
  bool e_phi_flag = true;
  float dot_phi = 0;
  float e_phi_d_last = 0;
  // yan
  int _flag = 0;
  double _wheel_angle_degree_last = 0;
  double _cal_wheel_angle = 0;
  int _record_cnt = 0;
  float _vehicle_wheel_angle;   // feedback steers
  float _keep_wheel_angle = 0;  // record last time of feedback steers
  float _delta_T = 0.05;

  // park
  float LLrear = 0;
  float KKrear = 0;

  float LLfront = 0;
  float KKfront = 0;
  vector<double> _path_x;
  vector<double> _path_y;
  vector<double> _path_theta;

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
  float _KK1 = 0;
  float _KK2 = 0;
  float _l1 = 0;
  float _l2 = 0;
  float _l3 = 0;
  float _l4 = 0;
  float _T = 0;

  // finalwaypoints gear flag
  float gear_flag_ = 0;

  // TODO::set follower flag
  bool follower_flag = false;

  /***/
  double lookahead_distance_ = 0;

  /*smooth filter**/
  vector<double> filterdata;

  // nav_msgs::Path Path;                         //路径点信息
  control_msgs::ego_pose vehicle_status;
  void controlInit();

  inline double deg2rad(double deg);

  /**
   * @brief follower
   * @param vehicle_x coordinate UTM_x(m)
   * @param vehicle_y coordinate UTM_y(m)
   * @param vehicle_heading_theta Yaw angle in map coordinate,start is X
   * axis,unclock (0~360). unit(rad)
   * @param _vehicle_v chassis speed (m/s)
   * @return  steerangle (deg)
   */
  double rear_park_control(float _vehicle_x, float _vehicle_y,
                           float _vehicle_heading_theta, float _vehicle_v);
  double front_park_control(float _vehicle_x, float _vehicle_y,
                            float _vehicle_heading_theta, float _vehicle_v);

  /**
   * @brief find cloest point in trajectory
   * @param _preview_x_rec
   * @param _preview_y_rec
   * @param count_min  cloestpoint index
   * @return cloest distance,left - right +
   */
  float find_min_point(float _preview_x_rec, float _preview_y_rec,
                       int *count_min);

  float limit_wheel_val(float _wheel_last_ref_rec,
                        float _wheel_angle_degree_rec);
  /**
   * @brief limit param in maxparam to minparam and change step
   * @
   */
  double Limitparam(float param, float param_old, float param_min,
                    float param_max, float param_step);
  /**
   * @brief limit param in maxparam to minparam
   *
   */
  double limitParamChangeUnit(double param, double max_param,
                              double min_param);  //限制幅值函数
  /**
   * @brief calculate curvature
   *
   */
  double CalculateCur(geometry_msgs::Point P1, geometry_msgs::Point P2,
                      geometry_msgs::Point P3);  //三点计算曲率
  double smoothfilter(double param,
                      int smoothsize);  //多点平滑滤波器参数为平滑量，平滑点数
  geometry_msgs::Point calcRelativeCoordinate(
      geometry_msgs::Point point_msg,
      geometry_msgs::Pose current_pose);  //坐标转换
  float transformcoordinate(float theta, int cloestnumber, float vehicle_x,
                            float vehicle_y);

  double qua_to_rpy(geometry_msgs::Quaternion posedata);
  geometry_msgs::Quaternion rpy_to_qua(double Yaw, double Pitch, double Roll);

 public:
  Parktrack_Control(/* args */);
  ~Parktrack_Control();
  /**
   * @name:
   * @brief:
   * @param {type}
   * @return desire steerangle  (deg)
   */
  void run_follower(float *out_steerangle);

  /**
   * @brief update vehicle chassis feedback data
   * coordinate(x,y,theta)(m,m,deg),speed(m/s),steerangle_rec(deg)
   *
   */
  void SetVehicleStatus(double _x, double _y, double _theta, double _speed,
                        double _vehicle_steerangle) {
    vehicle_status.x = _x;
    vehicle_status.y = _y;
    vehicle_status.yaw = _theta;

    vehicle_status.velocity = _speed;
    _vehicle_wheel_angle = _vehicle_steerangle;
    current_pose_flag = true;
    setVehiclestauts_flag = true;
  }
  void setWaypoints(const vector<control_msgs::Pathpoint> &msg_path) {
    // TODO::update Path
    // input::Path(x,y)
    _path_x.clear();
    _path_y.clear();
    _path_theta.clear();
    for (int i = 0; i < msg_path.size(); i++) {
      _path_x.push_back(msg_path[i].x);
      _path_y.push_back(msg_path[i].y);
      _path_theta.push_back(msg_path[i].yaw);
    }
    Waypoints_size = _path_x.size();
    waypoints_flag = true;
  }
};

}  // namespace Control

#endif  // PARKTRACK_CONTROL_H
