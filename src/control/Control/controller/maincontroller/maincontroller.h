/*
 * @Descripttion: novauto maincontroller contain parking lat_precontrolller  and
 * lon_PID_controlller
 * @version:
 * @Author: xiangzhang
 * @Date: 2020-09-03 23:03:49
 * @LastEditors: xiangzhang
 * @LastEditTime: 2020-10-12 15:18:30
 */
#pragma once

#include <algorithm>

#include "../../common/trajectorytool/trajectory_analyze.h"
#include "../pid_lon_controller/loncontrol.h"
#include "../pre_lat_controller/pre_lat_controller.h"
#include <mutex>

#define MAX_CURVATURE 0.2

namespace Control
{
  class maincontroller
  {
  private:
    // lock mutex
    std::mutex g_mutex_trajectory_lock;

    // global trajectory size
    int trajectory_size_ = 0.0;
    float desire_velocity = 0.0;
    float desire_velocity_last = 0.0;

    // define  lon_contorller and lat_controller
    LonControl _PI_loncontroller;
    Pre_Lat_Controller _Lat_controller;

    // def trajectorytool
    Trajectory_Analyze _Main_Trajectory_Analyze;
    std::vector<float> _path_yaw;
    std::vector<float> _path_curvature;
    std::vector<float> _patj_curvature_d;
    std::vector<float> _path_s;
    //receive struct define
    control_msgs::Pathpoint _current_pose_rec;
    control_msgs::Vehicle_status _vehicle_status_rec;
    vector<control_msgs::Pathpoint> _Pathpoints_rec;
    vector<control_msgs::Twistpoint> _Twistpoints_rec;

    // global struct message define
    control_msgs::Pathpoint _current_pose;
    control_msgs::Vehicle_status _vehicle_status;
    control_msgs::Vehicle_control _vehicle_control;

    //
    vector<control_msgs::Pathpoint> _Pathpoints;
    vector<control_msgs::Twistpoint> _Twistpoints;

    control_msgs::Vehicle_control _vehicle_control_req;
    //RESTE if stop and human mode
    bool maincontroller_reset();

  public:
    /* define funtion */
    /**
   * @name:
   * @brief: exec function
   * @param
   * @return control instructions
   */
    void maincontroller_run();

    /**
   * @name:
   * @brief: get IMU and GPS data
   * @param {type}
   * @return {type}
   */
    void get_localization(control_msgs::Pathpoint localization_point)
    {
      _current_pose_rec = localization_point;
    }
    /**
   * @name:
   * @brief: get vehicle chassis data
   * @param {type}
   * @return {type}
   */
    void get_vehicle_status(control_msgs::Vehicle_status vehicle_status)
    {
      _vehicle_status_rec = vehicle_status;
    }
    /**
   * @name:
   * @brief: send vehicle control message
   * @param {type}
   * @return {type}
   */
    void send_vehicle_instruction(
        control_msgs::Vehicle_control *vehicle_control)
    {
      *vehicle_control = _vehicle_control_req;
    }
    /**
   * @name:
   * @brief: get trajectory and separate Pah(x,y.yaw) and velocity
   * (m/s)sequence()
   * @param {type}
   * @return {type}
   */
    void get_trajectory(const vector<control_msgs::Trajectorypoint> &trajectory)
    {
      lock_guard<mutex> lock(g_mutex_trajectory_lock);
      _Pathpoints_rec.clear();
      _Twistpoints_rec.clear();
      std::vector<std::pair<float, float>> _path_trajectory;
      std::pair<float, float> _path_trajectory_point;
      trajectory_size_ = trajectory.size();
      for (int i = 0; i < trajectory_size_; i++)
      {
        _Pathpoints_rec.push_back(trajectory[i].pose);
        _Twistpoints_rec.push_back(trajectory[i].twist);

        _path_trajectory_point.first = trajectory[i].pose.x;
        _path_trajectory_point.second = trajectory[i].pose.y;
        _path_trajectory.push_back(_path_trajectory_point);
      }
      cout << " trajectory_size_   " << trajectory_size_ << endl;
      // TODO::trajectory_anazlyer 计算道路的航向角，曲率，曲率变化率，长度
      _Main_Trajectory_Analyze.computePathProfile(_path_trajectory, &_path_yaw,
                                                  &_path_s, &_path_curvature,
                                                  &_patj_curvature_d);
      // for (int i = 0; i < _path_curvature.size() && i < _path_curvature.size(); i++)
      // {
      //   _Pathpoints_rec[i].curvature = _path_curvature[i];
      //   cout << " get curvature " << _path_curvature[i] << endl;
      //   ofstream out;
      //   out.open("/home/fangniuwa/g3bug_fix/control_fix/src/Control/kappa.txt", std::ios::out | std::ios::app);
      //   out << setiosflags(ios::fixed) << setprecision(3) << _path_curvature[i] << endl;
      //   out.close();
      // }
    }
    /**
   * @name:
   * @brief: velocity unit(km/h -> m/s)
   * @param {type}
   * @return {type}
   */
    float
    velocity_km_to_m(float velocity_);
    /**
   * @name:
   * @brief:velocity unit(m/s -> km/h)
   * @param {type}
   * @return {type}
   */
    float velocity_m_to_km(float velocity_);
    maincontroller(/* args */);
    ~maincontroller();
  }; // namespace Control

} // namespace Control
