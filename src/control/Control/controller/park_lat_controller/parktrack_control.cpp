
/*author :zx
date 2020.2.25
function ： test Vehicle mode
*/
#include "parktrack_control.h"

#include <algorithm>

using namespace std;
using namespace Eigen;

namespace Control {
Parktrack_Control::Parktrack_Control()

{
  /**init param of control and use yaml  config*/
  controlInit();
}

Parktrack_Control::~Parktrack_Control() {}

template <typename T>
T getParam(const YAML::Node &node, const string &name, const T &defaultValue) {
  T v;
  try {
    v = node[name].as<T>();
    std::cout << "Found parameter: " << name << ",\tvalue: " << v << std::endl;
  } catch (std::exception e) {
    v = defaultValue;
    std::cout << "Cannot find parameter: " << name
              << ",\tassigning  default: " << v << std::endl;
  }
  return v;
}
#ifndef MAX_PATH
#define MAX_PATH 260
#endif
/**Init control param ,use yaml file*/
void Parktrack_Control::controlInit() {
  char config_path[MAX_PATH] = {0};

  if (getenv("MINI_AUTO_ROOT") == NULL)
    ROS_ERROR(
        "Not find env (MINI_AUTO_ROOT)!!!!  Please set MINI_AUTO_ROOT={your "
        "mini_auto absolute path},like "
        "export=MINI_AUTO_ROOT=/home/username/mini_auto");

  strcpy(config_path, getenv("MINI_AUTO_ROOT"));
  strcat(config_path, "/src/control/review_code/conf/park_cfg.yaml");

  YAML::Node dset_config = YAML::LoadFile(config_path);

  // Vehiclemodel,Controlparam two key_value
  YAML::Node Vehiclemodel = dset_config["Vehiclemodel"];
  YAML::Node Controlparam = dset_config["Controlparam"];

  /****VehicleModle*****/
  delta_f = getParam<float>(Vehiclemodel, "_delta_f", 0);  //前轮偏角度
  K = getParam<float>(Vehiclemodel, "_K", 0);              //方向盘传动比
  LLrear = getParam<float>(Vehiclemodel, "LLrear", 0);     //最小转向半径
  KKrear = getParam<float>(Vehiclemodel, "KKrear", 0);     //方向盘传动比
  LLfront = getParam<float>(Vehiclemodel, "LLfront", 0);   //最小转向半径
  KKfront = getParam<float>(Vehiclemodel, "KKfront", 0);   //方向盘传动比

  /********Controlparam***************/
  _smoothsteernum = getParam<float>(Controlparam, "_smoothsteernum", 0);
}
// ////////////////////////////////主程序////////////////////////////////////////////////////////////
/***********************************************************************************************/
void Parktrack_Control::run_follower(float *out_steerangle) {
  if (gear_flag_ >= 0)
    delta_f = front_park_control(vehicle_status.x, vehicle_status.y,
                                 ((vehicle_status.yaw) / 180) * PI,
                                 vehicle_status.velocity);  //
  else
    delta_f = rear_park_control(vehicle_status.x, vehicle_status.y,
                                ((vehicle_status.yaw) / 180) * PI,
                                vehicle_status.velocity);  //
  delta_f = smoothfilter(delta_f, _smoothsteernum);
  *out_steerangle = delta_f;
  delta_f_old = delta_f;
}
double Parktrack_Control::Limitparam(float param, float param_old,
                                     float param_min, float param_max,
                                     float param_step) {
  if ((param - param_old) > param_step)
    param = param_old + param_step;
  else if ((param - param_old) < -param_step)
    param = param_old - param_step;
  else
    param = param;
  if (param > param_max)
    param = param_max;
  else if (param < param_min)
    param = param_min;
  else
    param = param;
  return param;
}
double Parktrack_Control::smoothfilter(double param, int smoothsize) {
  // TODO::smooth filter
  // input:: smooth target param, the number of smoothsize
  // output:: parameter after smooth

  if (filterdata.size() >= smoothsize) {
    for (int i = 0; i < smoothsize - 1; i++) {
      filterdata[i] = filterdata[i + 1];
    }
    filterdata[smoothsize - 1] = param;
  } else {
    filterdata.push_back(param);
  }
  double sum =
      std::accumulate(std::begin(filterdata), std::end(filterdata), 0.0);
  double mean = sum / filterdata.size();  //均值
  return mean;
}
double Parktrack_Control::rear_park_control(float _vehicle_x, float _vehicle_y,
                                            float _vehicle_heading_theta,
                                            float _vehicle_v) {
  static float LL = 0;  // distance from current position to lookahead point,
  static float e_y = 0, e_phi_d = 0;
  static float u = 0;
  static float KK = 0;
  // TODO:: !!!!!!!!!need to change find_min_point function
  // preview
  _vehicle_x = _vehicle_x + 0.1 * cos(_vehicle_heading_theta);
  _vehicle_y = _vehicle_y + 0.1 * sin(_vehicle_heading_theta);

  // carla coordinte point is geometry centre
  float _vehicle_x_rear = _vehicle_x - 0.3 * cos(_vehicle_heading_theta);
  ;
  float _vehicle_y_rear = _vehicle_y - 0.3 * sin(_vehicle_heading_theta);
  ;

  //
  float _vehicle_x_front = _vehicle_x + 2.6 * cos(_vehicle_heading_theta);
  float _vehicle_y_front = _vehicle_y + 2.6 * sin(_vehicle_heading_theta);

  // control input::当前点侧向误差, 当前点的路径航向,航向偏差

  // TODO:: 当前侧向偏差求解,航向角偏差求解模块
  int nextpoint_num = 0;

  find_min_point(_vehicle_x_rear, _vehicle_y_rear, &nextpoint_num);

  //使用最近距离求侧向偏差,车在路径左偏差为负,右为正
  // float e_y2 = find_min_point(_vehicle_x, _vehicle_y, &nextpoint_num);

  //使用坐标转换求解e_y,车辆坐标系为右手坐标系,
  float _vehicle_heading_theta_rear = _vehicle_heading_theta + PI;
  e_y = transformcoordinate(_vehicle_heading_theta_rear, nextpoint_num,
                            _vehicle_x_rear, _vehicle_y_rear);
  e_y = limitParamChangeUnit(e_y, 1.5, -1.5);
  // TODO::计算车辆当前位置与最近的路径点航向角偏差
  static float path_yaw = 0;
  static float phi_error = 0;
  static float phi_d = 0;
  static float phi = 0;
  int Yaw_nextpoint_num = 0;
  find_min_point(_vehicle_x_rear, _vehicle_y_rear, &Yaw_nextpoint_num);
  if (Yaw_nextpoint_num < _path_x.size() - 10) {
    phi_d = _path_theta[Yaw_nextpoint_num + 10] / 180 * PI;
  } else {
    phi_d = _path_theta[_path_x.size() - 1] / 180 * PI;
  }
  phi = _vehicle_heading_theta_rear;
  double ys = fmod(((phi - phi_d) + PI), (2 * PI));
  if (ys < 0) ys = ys + 2 * PI;
  e_phi_d = 0 - (ys - PI);

  // TODO::计算横摆角速度和期望角速度偏差
  //测量值从oxfordIMU获取
  double kappa = 0;
  if (nextpoint_num < _path_x.size()) {
    geometry_msgs::Point p1, p2, p3;
    if (nextpoint_num < (_path_x.size() - 20)) {
      p1.x = _path_x[nextpoint_num + 1];
      p2.x = _path_x[nextpoint_num + 10];
      p3.x = _path_x[nextpoint_num + 20];
      p1.y = _path_y[nextpoint_num + 1];
      p2.y = _path_y[nextpoint_num + 10];
      p3.y = _path_y[nextpoint_num + 20];
    } else
      // ROS_WARN("three point kappa is the lastpoint!!!");

      kappa = CalculateCur(p1, p2, p3) + 0.000001;
  } else {
    ROS_INFO("path end");
  }

  //判断方向时,使用的预瞄点和车辆当前点判断曲率方向.
  float _d_dis_k = 0.7 * _vehicle_v;
  float final_distance =
      sqrt(pow((_vehicle_x - _path_x[Waypoints_size - 1]), 2) +
           pow((_vehicle_y - _path_y[Waypoints_size - 1]), 2));
  if (final_distance < 5)
    ;
  _d_dis_max_k = final_distance;
  _d_dis_k = limitParamChangeUnit(_d_dis_k, _d_dis_max_k, _d_dis_min_k);
  double _x_preview_cur =
      _vehicle_x + _d_dis_k * cos(_vehicle_heading_theta_rear);
  double _y_preview_cur =
      _vehicle_y + _d_dis_k * sin(_vehicle_heading_theta_rear);

  // kappa puresuit method (l*l)/(2*x)
  float kappa_pure = 0;
  double kappa_min = 1 / 9e10;
  geometry_msgs::Point pre_point;
  geometry_msgs::Pose current_pose_;
  int pre_number = 0;
  int near_number = 0;
  find_min_point(_x_preview_cur, _y_preview_cur, &pre_number);
  pre_point.x = _path_x[pre_number];
  pre_point.y = _path_y[pre_number];
  pre_point.z = 0;
  find_min_point(_vehicle_x, _vehicle_x, &near_number);

  current_pose_.position.x = _vehicle_x;
  current_pose_.position.y = _vehicle_y;
  current_pose_.position.z = 0;
  current_pose_.orientation = rpy_to_qua(_vehicle_heading_theta_rear, 0, 0);

  double denominator =
      pow((pre_point.x - _vehicle_x), 2) + pow((pre_point.y - _vehicle_y), 2);
  double numerator = 2 * calcRelativeCoordinate(pre_point, current_pose_).y;

  if (denominator != 0)
    kappa_pure = numerator / denominator;
  else {
    if (numerator > 0)
      kappa_pure = kappa_min;
    else
      kappa_pure = -kappa_min;
  }
  kappa = kappa_pure;

  KK = KKrear;
  LL = LLrear;

  u = KK * e_y + LL * (e_phi_d / PI * 180);
  float max_u = 700;
  u = limitParamChangeUnit(u, max_u, -max_u);
  u = u / 180 * PI;
  cout << " e_Y  e_phi_d " << e_y << " " << e_phi_d << " " << phi_d << " "
       << phi << " " << u / PI * 180 << " " << KK * e_y << " "
       << LL * (e_phi_d / PI * 180) << endl;

  return u;  //
}
double Parktrack_Control::front_park_control(float _vehicle_x, float _vehicle_y,
                                             float _vehicle_heading_theta,
                                             float _vehicle_v) {
  static float LL = 0;  // distance from current position to lookahead point,
  static float e_y = 0, e_phi_d = 0;
  static float u = 0;
  static float KK = 0;
  // TODO:: !!!!!!!!!need to change find_min_point function
  // preview
  _vehicle_x = _vehicle_x + 0.1 * cos(_vehicle_heading_theta);
  _vehicle_y = _vehicle_y + 0.1 * sin(_vehicle_heading_theta);

  // carla coordinte point is geometry centre
  float _vehicle_x_rear = _vehicle_x - 0 * cos(_vehicle_heading_theta);
  float _vehicle_y_rear = _vehicle_y - 0 * sin(_vehicle_heading_theta);

  //
  float _vehicle_x_front = _vehicle_x + 2.6 * cos(_vehicle_heading_theta);
  float _vehicle_y_front = _vehicle_y + 2.6 * sin(_vehicle_heading_theta);

  // control input::当前点侧向误差, 当前点的路径航向,航向偏差

  // TODO:: 当前侧向偏差求解,航向角偏差求解模块
  int nextpoint_num = 0;

  find_min_point(_vehicle_x_rear, _vehicle_y_rear, &nextpoint_num);

  //使用最近距离求侧向偏差,车在路径左偏差为负,右为正
  // float e_y2 = find_min_point(_vehicle_x, _vehicle_y, &nextpoint_num);

  //使用坐标转换求解e_y,车辆坐标系为右手坐标系,
  e_y = transformcoordinate(_vehicle_heading_theta, nextpoint_num,
                            _vehicle_x_front, _vehicle_y_front);
  // e_y = limitParamChangeUnit(e_y, 1.5, -1.5);

  // TODO::计算车辆当前位置与最近的路径点航向角偏差
  static float path_yaw = 0;
  static float phi_error = 0;
  static float phi_d = 0;
  static float phi = 0;
  int Yaw_nextpoint_num = 0;
  find_min_point(_vehicle_x_front, _vehicle_x_front, &Yaw_nextpoint_num);
  // cout << "Yaw_nextpoint_num  " << Yaw_nextpoint_num << " " << _path_x.size()
  // << endl;
  if (Yaw_nextpoint_num < _path_x.size())
    phi_d = _path_theta[Yaw_nextpoint_num + 10] / 180 * PI;
  else
    phi_d = _path_theta[_path_x.size() - 1] / 180 * PI;
  phi = _vehicle_heading_theta;
  double ys = fmod(((phi - phi_d) + PI), (2 * PI));
  if (ys < 0) ys = ys + 2 * PI;
  e_phi_d = 0 - (ys - PI);
  if (!e_phi_flag) dot_phi = (e_phi_d - e_phi_d_last) / 0.05;
  if (e_phi_flag) e_phi_flag = false;

  //测量值从oxfordIMU获取
  double kappa = 0;
  if (nextpoint_num < _path_x.size()) {
    geometry_msgs::Point p1, p2, p3;
    if (nextpoint_num < (_path_x.size() - 20)) {
      p1.x = _path_x[nextpoint_num + 1];
      p2.x = _path_x[nextpoint_num + 10];
      p3.x = _path_x[nextpoint_num + 20];
      p1.y = _path_y[nextpoint_num + 1];
      p2.y = _path_y[nextpoint_num + 10];
      p3.y = _path_y[nextpoint_num + 20];
    } else
      // ROS_WARN("three point kappa is the lastpoint!!!");

      kappa = CalculateCur(p1, p2, p3) + 1e-6;
  } else {
    ROS_INFO("path end");
  }

  //判断方向时,使用的预瞄点和车辆当前点判断曲率方向.
  float _d_dis_k = _t_preview * _vehicle_v + _d_0;
  _d_dis_k = limitParamChangeUnit(_d_dis_k, _d_dis_max_k, _d_dis_min_k);
  double _x_preview_cur = _vehicle_x + _d_dis_k * cos(_vehicle_heading_theta);
  double _y_preview_cur = _vehicle_y + _d_dis_k * sin(_vehicle_heading_theta);

  geometry_msgs::Point pre_point;
  geometry_msgs::Pose current_pose_;
  int pre_number = 0;
  int near_number = 0;
  find_min_point(_x_preview_cur, _y_preview_cur, &pre_number);
  if (pre_number < _path_x.size() && nextpoint_num < _path_x.size()) {
    // cout<<" prenumber  "<<pre_number<<endl;
    pre_point.x = _path_x[pre_number];
    pre_point.y = _path_y[pre_number];
    pre_point.z = 0;
    find_min_point(_vehicle_x, _vehicle_x, &near_number);
    current_pose_.position.x = _path_x[nextpoint_num];
    current_pose_.position.y = _path_y[nextpoint_num];
    current_pose_.position.z = 0;
    current_pose_.orientation = rpy_to_qua(phi_d, 0, 0);

    double kappa_pure;
    double denominator = pow((pre_point.x - vehicle_status.x), 2) +
                         pow((pre_point.y - vehicle_status.y), 2);
    double numerator = 2 * calcRelativeCoordinate(pre_point, current_pose_).y;

    if (denominator != 0)
      kappa_pure = numerator / denominator;
    else {
      if (numerator > 0)
        // kappa_pure = kappa_min;
        kappa = kappa;
      else
        // kappa_pure = -kappa_min;
        kappa = -kappa;
    }
  } else {
    ROS_INFO("path end ");
  }

  KK = KKfront;
  LL = LLfront;
  u = KK * e_y + LL * (e_phi_d);
  // cout << " KK * e_y    " << e_y << " LL * (e_phi_d) " << e_phi_d /M_PI*180
  // <<" dot_phi "<<dot_phi<< endl;
  float max_u = 600 / 180 * PI;
  u = limitParamChangeUnit(u, max_u, -max_u);

  e_phi_d_last = e_phi_d_last;
  return u;  //
}

float Parktrack_Control::transformcoordinate(float theta, int cloestnumber,
                                             float vehicle_x, float vehicle_y) {
  //逆时针旋转theta 为正
  //车身坐标系为右手坐标系
  Eigen::Matrix2d transform;
  static int startnumber = 0;
  static int endnumber = 0;
  vector<float> transform_path_x;
  vector<float> transform_path_y;
  float transform_x = 0;
  float transform_y = 0;
  transform_path_x.clear();
  transform_path_y.clear();

  transform << cos(theta), -sin(theta), sin(theta), cos(theta);

  if (cloestnumber < 50)
    startnumber = 0;
  else
    startnumber = cloestnumber - 50;
  if (cloestnumber > (_path_x.size() - 50))
    endnumber = _path_x.size();
  else
    endnumber = cloestnumber + 50;
  if (startnumber < endnumber) {
    for (int i = startnumber; i < endnumber; i++) {
      //左右手坐标系不一样吗
      transform_x = (_path_x[i] - vehicle_x) * cos(theta) +
                    sin(theta) * (_path_y[i] - vehicle_y);
      transform_y = (_path_x[i] - vehicle_x) * -1 * sin(theta) +
                    cos(theta) * (_path_y[i] - vehicle_y);
      transform_x = abs(transform_x);
      transform_path_x.push_back(transform_x);
      transform_path_y.push_back(transform_y);
    }
    auto smallest = std::min_element(std::begin(transform_path_x),
                                     std::end(transform_path_x));
    int smallest_index = std::distance(std::begin(transform_path_x), smallest);

    return transform_path_y[smallest_index];
  } else {
    ROS_ERROR("final waypoint transform is error !! ");
  }
}
geometry_msgs::Point Parktrack_Control::calcRelativeCoordinate(
    geometry_msgs::Point point_msg, geometry_msgs::Pose current_pose) {
  // TODO:: calculation relative coordinate of point from current_pose frame
  // input:: taget preview point,current pose of vehicle
  // output:: transform coordinate

  tf::Transform inverse;
  tf::poseMsgToTF(current_pose, inverse);
  tf::Transform transform = inverse.inverse();

  tf::Point p;
  pointMsgToTF(point_msg, p);
  tf::Point tf_p = transform * p;
  geometry_msgs::Point tf_point_msg;
  pointTFToMsg(tf_p, tf_point_msg);
  return tf_point_msg;
}

float Parktrack_Control::find_min_point(float _preview_x_rec,
                                        float _preview_y_rec, int *count_min) {
  // TODO::find closest point error (left - right)
  // input:: coordinate (m,m)
  // outpu:: the index of closest point

  float _preview_x_tmp;
  float _preview_y_tmp;
  float _L_min = 20.0;
  float _L;
  float _x_min = -1.0;
  float _y_min = -1.0;
  float _Oss_tmp = 0;
  int _count_min = 0;
  _preview_x_tmp = _preview_x_rec;
  _preview_y_tmp = _preview_y_rec;
  float _vehicle_x_ref = vehicle_status.x;
  float _vehicle_y_ref = vehicle_status.y;
  for (int _path_ii = 0; _path_ii < _path_x.size(); _path_ii++) {
    _L = sqrt(pow((_preview_x_tmp - _path_x[_path_ii]), 2) +
              pow((_preview_y_tmp - _path_y[_path_ii]), 2));
    if (_L <= _L_min) {
      _L_min = _L;
      _x_min = _path_x[_path_ii];
      _y_min = _path_y[_path_ii];
      _count_min = _path_ii;
    } else {
      _L_min = _L_min;
      _x_min = _x_min;
      _y_min = _y_min;
      _count_min = _count_min;
    }
  }
  // TODO:add closest point index
  *count_min = _count_min;
  if ((_preview_x_tmp - _vehicle_x_ref) * (_y_min - _vehicle_y_ref) -
          (_preview_y_tmp - _vehicle_y_ref) * (_x_min - _vehicle_x_ref) >=
      0)
    _Oss_tmp = _L_min;
  else {
    _Oss_tmp = -1.0 * _L_min;
  }
  return _Oss_tmp;
}

double Parktrack_Control::CalculateCur(geometry_msgs::Point P1,
                                       geometry_msgs::Point P2,
                                       geometry_msgs::Point P3) {
  // TODO::calculate cur
  // input::three points
  // output:: cur,  `
  double curvity_pre = 0;  // last time cur
  double curvity = 0;      // current cur
  double curvity_R = 0;    // current radius
  double speed_pre = 0;    // last speed_path_theta
  // judge three point not in same line
  if (P1.x == P2.x == P3.x || P1.y == P2.y == P3.y) {
    curvity = 0;
  } else {
    if (P1.x == P2.x && P1.y == P2.y || P1.x == P3.x && P1.y == P3.y ||
        P2.x == P3.x && P2.y == P3.y) {
      // cout<<"path have same point!"<<endl;
    } else {
      double dis1, dis2, dis3;
      double cosA, sinA, dis;
      dis1 =
          sqrt((P1.x - P2.x) * (P1.x - P2.x) + (P1.y - P2.y) * (P1.y - P2.y));
      dis2 =
          sqrt((P1.x - P3.x) * (P1.x - P3.x) + (P1.y - P3.y) * (P1.y - P3.y));
      dis3 =
          sqrt((P2.x - P3.x) * (P2.x - P3.x) + (P2.y - P3.y) * (P2.y - P3.y));
      dis = dis1 * dis1 + dis3 * dis3 - dis2 * dis2;
      cosA = dis / (2 * dis1 * dis3) + 1e-6;
      if (cosA > 1) {
        cosA = 1;
      }
      sinA = sqrt(1 - cosA * cosA);
      curvity_R = 0.5 * dis2 / sinA;
      curvity = 1 / curvity_R;
    }
  }
  return curvity;
}

double Parktrack_Control::limitParamChangeUnit(double param, double max_param,
                                               double min_param) {
  // TODO::limit parameter boundary
  // input::limit param,max and min param
  // output:: param after limit
  if (param > max_param) {
    param = max_param;
  } else if (param < min_param) {
    param = min_param;
  } else {
    param = param;
  }

  return param;
}
/*四元素转航向角*/
double Parktrack_Control::qua_to_rpy(geometry_msgs::Quaternion posedata) {
  // TODO:: oritation transform to RPY
  // input::oritation
  // output::Yaw angle(deg)
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
// RPY转四元素//
geometry_msgs::Quaternion Parktrack_Control::rpy_to_qua(double Yaw,
                                                        double Pitch,
                                                        double Roll) {
  // TODO:: RPY transform to oritation
  // input::RPY(deg)
  // output::oritation

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

}  // namespace Control
