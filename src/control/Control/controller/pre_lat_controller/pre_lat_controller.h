/*
 * @Descripttion:
 * @version:
 * @Author: xiangzhang
 * @Date: 2020-09-01 19:07:33
 * @LastEditors: xiangzhang
 * @LastEditTime: 2020-10-20 16:40:02
 */

#pragma once

#include <stdio.h>

#include <cmath>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <map>
#include <numeric>
#include <thread>

// inlcude iostream and string libraries
#include <time.h>
#include <yaml-cpp/yaml.h>
#include <mutex>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/QR>
#include <string>
#include <vector>
// include control
#include "../../common/filter/limitfilter.h"
#include "../../common/filter/smoothfilter.h"
#include "../../common/filter/mean_filter.h"
#include "../../common/math/coordinate_tranform.h"
#include "../../common/math/interpolation_1d.h"
#include "../message/control_msgs.h"

using namespace Eigen;
using namespace std;

namespace Control
{
  class Pre_Lat_Controller
  {
  private:
    //locxk mutex
    std::mutex g_mutex_pre_trajectory;
    std::mutex g_mutex_pre_chassis_and_pose;
    // vehicle mode param
    double lf = 1.5; //
    double lr = 1.5; //
    double L = lr + lf;
    double m = 1818.2; // mass of Vehicle
    double Ccf = 66900;
    double Ccr = 62700;
    double Clf = 66900;
    double Clr = 62700;
    double delta_f = 0;
    double delta_f_old = 0;
    double Sf = 0.2;
    double Sr = 0.2;
    double I = 4175;
    double wheel_base = 2.7;
    double g = 9.8;
    double R_min = 5.13; //最小转向半径
    double K = 15.7;     //方向盘传动比

    // yan
    int _flag = 0;
    double _wheel_angle_degree_last = 0;
    double _cal_wheel_angle = 0;
    int _record_cnt = 0;
    double _vehicle_wheel_angle;  // feedback steers
    double _keep_wheel_angle = 0; // record last time of feedback steers
    double _delta_T = 0.05;

    //meanfilter
    MeanFilter lat_error_filter;
    MeanFilter heading_error_filter;
    MeanFilter kappa_filter;
    MeanFilter kappa_pure_filter;
    MeanFilter heading_ref_filter;
    //Interpolation
    std::unique_ptr<Interpolation1D> lat_err_interpolation_;
    std::unique_ptr<Interpolation1D> heading_err_interpolation_;

    std::vector<double> _path_x_rec;
    std::vector<double> _path_y_rec;
    std::vector<double> _path_theta_rec;
    std::vector<double> _path_curvature_rec;

    std::vector<double> _path_x;
    std::vector<double> _path_y;
    std::vector<double> _path_theta;
    std::vector<double> _path_curvature;
    std::vector<double> _path_s;
    // parameters of preview
    double _p_post = 14.5;

    double _lat_gain = 0;
    double _lat_angle_gain = 0;
    double curvature_gain = 0;
    double pre_K = 0;              // the weight of preview control
    double feedforward_K = 0;      //  Curvature feed forward gravity
    double feedforward_K_last = 0; //  （pre_K+feedforward_K=1）
    double feedforward_P = 0;      // yumiaowucha

    double _distance_pre = 0;
    double _dis_pre_max = 20;             // maxmum lookahead distance
    double _dis_pre_min = 2;              // minimum lookahead distance
    double _wheel_angle_degree_max = 470; // limit maxmum steer angle
    double _wheel_angle_degree_sec = 200; // limit max steerangle in one sec
    double _t_preview = 2.8;              // preview time 1
    double _t_preview_turn = 0;           // preview time 2
    double preview_time = 0;              //time for vehicle chassis delay
    double _kappa = 0;                    // Curvature compensation coefficient

    double lat_error_max = 1.5;
    double yaw_error_max = 30; // deg
    double PathLength_min = 6;
    //
    int Waypoints_size = 0;
    double PathLength = 0;

    /**subdata flag*/
    bool current_pose_flag = false;
    bool waypoints_flag = false;
    bool setVehiclestauts_flag = false;
    int _keep_steer = 0;

    // Yaml add parameters
    double _smoothsteernum = 0;
    int path_size_min = 0; // pathsize mininuber

    double kappa_min = 1 / 9e10; // Yamlparam
    double _d_0 = 1.957;         //起始距离.Yamlparam
    double _d_dis_min = 2.5;
    double _d_dis_max = 40;
    double _d_dis_min_k = 5;
    double _d_dis_max_k = 40;
    double kappa_path_change_flag = 0.02;
    /**/
    bool follower_flag = false;

    /*smooth filter**/
    vector<double> filterdata;
    vector<double> filterdata_lat_error;
    vector<double> filterdata_angle_error;
    vector<double> filterdata_kappa_pure;
    vector<double> filterdata_kappa_trajectory;
    /**/
    double _x_preview_rviz; // the x coordinates of preview point
    double _y_preview_rviz; // the y coordinates of preview point

    //
    control_msgs::Vehicle_status _vehicle_chassis_fd_rec;
    control_msgs::Pathpoint _current_pose_rec;
    control_msgs::Vehicle_status _vehicle_chassis_fd;
    control_msgs::Pathpoint _current_pose;

    void controlInit();

    // control funtions
    double ControlStr(double _vehicle_heading_theta, double _vehicle_v,
                      double _vehicle_x, double _vehicle_y);
    // add zx
    bool find_cloest_point_index(const std::vector<double> &path_x,
                                 const std::vector<double> &path_y,
                                 double position_x, double position_y,
                                 int *count_min);

    bool find_cloest_point_dis(const std::vector<double> &path_x,
                               const std::vector<double> &path_y, double positon_x,
                               double positon_y, double positon_theta,
                               double *dis_min);

    bool find_cloest_point_coordinate(const std::vector<double> &path_x,
                                      const std::vector<double> &path_y,
                                      const int cloest_index,
                                      const control_msgs::Point &now_pose,
                                      control_msgs::Point *cloest_pathpoint);

    double cal_transfer_val(double _d_dis_rec, double _vehicle_v);

    double limit_wheel_val(double _wheel_last_ref_rec,
                           double _wheel_angle_degree_rec);

    double limitParamChangeUnit(double param, double max_param, double min_param);

    double CalculateCur(control_msgs::Point P1, control_msgs::Point P2,
                        control_msgs::Point P3);
    void selectCoefficient(double v_s, double kappa_s, double *G);

    //计算路径长度和每两个点的间距,判断是否正常,是否需要插值.
    bool Pathlength(const std::vector<double> &path_x,
                    const std::vector<double> &path_y, double *pathlength,
                    std::vector<double> *path_s);
    //两点计算恒等式系数
    // let the linear equation be "ax + by + c = 0"
    // if there are two points (x1,y1) , (x2,y2), a = "y2-y1, b = "(-1) * x2 - x1"
    // ,c = "(-1) * (y2-y1)x1 + (x2-x1)y1"
    bool getLinearEquation(control_msgs::Point start, control_msgs::Point end,
                           double *a, double *b, double *c);
    //计算点和直线的距离
    double getDistanceBetweenLineAndPoint(control_msgs::Point point, double a,
                                          double b, double c);

    bool LoadLatGainScheduler();
    //相互垂直的交点坐标
    bool getcloestPointCoorindate(control_msgs::Point Point_P, control_msgs::Point *PointonLine, double a,
                                  double b, double c1);

  public:
    Pre_Lat_Controller(/* args */);
    ~Pre_Lat_Controller();

    void run_follower(float *out_steerangle);

    // coordinate(x,y,theta)(m,m,deg),speed(m/s),steerangle_rec(deg)
    void SetVehicleStatus(const control_msgs::Vehicle_status &chassis_msg, const control_msgs::Pathpoint &current_pose_msg)
    {
      lock_guard<mutex> lock(g_mutex_pre_chassis_and_pose);
      _vehicle_chassis_fd_rec = chassis_msg;
      _current_pose_rec = current_pose_msg;
      current_pose_flag = true;
      setVehiclestauts_flag = true;
    }

    //使用control_msgs
    void setWaypoints(const vector<control_msgs::Pathpoint> &msg_path)
    {
      lock_guard<mutex> lock(g_mutex_pre_trajectory);
      // TODO::update Path
      // input::Path(x,y)
      _path_x_rec.clear();
      _path_y_rec.clear();
      _path_theta_rec.clear();
      _path_curvature_rec.clear();
      for (int i = 0; i < msg_path.size(); ++i)
      {
        _path_x_rec.push_back(msg_path[i].x);
        _path_y_rec.push_back(msg_path[i].y);
        _path_theta_rec.push_back(msg_path[i].yaw);
        _path_curvature_rec.push_back(msg_path[i].curvature);
      }
      Waypoints_size = _path_x_rec.size();
      waypoints_flag = true;
    }
    /*out preview point show in rviz*/
    void sendXYpre(double *x, double *y, double *error)
    {
      // TODO::update Path
      // input::Path(x,y)
      *x = _x_preview_rviz;
      *y = _y_preview_rviz;
    }
  };

} // namespace Control
