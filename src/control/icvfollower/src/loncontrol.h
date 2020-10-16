
#ifndef LONCONTROL_H
#define LONCONTROL_H
//*init longitude calss*/

#include <numeric>
#include <queue>
#include <vector>

#include "../include/control_msgs.h"
#include "nav_msgs/Path.h"
#include "pid_controller.h"
#include "ros/ros.h"

using namespace std;
namespace Control {

class LonControl {
 private:
  /*PIDController*/
  PIDController speed_pid_controller_;
  PIDController position_pid_controller_;

  /* data   */
  float target_v;                // desire velocity
  float fbk_v;                   // chassis's  velocity
  float fbk_acc;                 // chassis's  accerlation
  float target_acc;              // desire acc
  bool Pathlength_flag = false;  // judge path end

  float speed_error = 0;
  float speed_compenzation_error = 0;
  float target_acc_old = 0;
  float dead_throttle = 0;

  float target_acc_min = -20;
  float target_acc_max = 20;
  float target_acc_step = 0.01;
  vector<float> filterdata;
  float dt = 0.02;

  // Yaml parameter defined
  int target_acc_smoothnumber = 0;
  float Kp = 0;
  float Ki = 0;
  float Kd = 0;
  float speed_error_max = 0;
  float speed_error_min = 0;
  float integrator_saturation_max = 0;
  float integrator_saturation_min = 0;

  float out_acc_max = 2;
  float out_acc_min = -2;
  float out_acc_rate_max = 5;
  float out_acc_rate_min = -5;

  // loncontrol limit parameters
  float acc_boundary_max = 0;
  float acc_boundary_min = 0;
  float jerk_boundary_max = 0;
  float jerk_boundary_min = 0;

  // Path vector
  vector<control_msgs::Pathpoint> trajectory;
  control_msgs::Pathpoint vehicle;
  /**
   * @brief calculate speed_error
   * @param target_v(m/s)
   * @param fbk_v(m/s)
   */
  float GetVelocityError(const float ref_v, const float chassis_v);

  /**
   * @brief calculate target acc
   * @param speed_control_error speed_error (m/s)
   * @return target_acc (m/s2)
   */
  float VelocityControl(float speed_control_error);

  /**
   * @brief limit param range
   * @param param now param
   * @param param_old last t-1 time param
   * @param param_min minst param
   * @param param_max max param
   * @param param_step Periodic change in step size
   */
  float Limitparam(float param, float param_old, float param_min,
                   float param_max, float param_step);
  /**
   * @brief claclulate param's means
   * @param param target param
   * @param datanumber number of filterdata
   */
  float smoothfilter(float param, int datanumber);
  /**
   * @brief Init loncontrol param
   * @param param target param
   * @param datanumber number of filterdata
   */
  void lon_control_init();
  float limit_param(float param, float param_last, float param_min,
                    float param_max, float param_max_rate, float param_min_rate,
                    float dt);

 public:
  // input cycle time
  LonControl();
  ~LonControl();
  void Loncontrolrun();

  // vehicle coordinate
  void getvehiclecoordinate(float vehicle_x, float vehicle_y,
                            float vehicle_yaw) {
    vehicle.x = vehicle_x;
    vehicle.y = vehicle_y;
    vehicle.yaw = vehicle_yaw;
  }
  // chssis datas input
  void inputcontroldata(float target_chassis_v, float fbk_chassis_v,
                        float fbk_chassis_acc) {
    target_v = target_chassis_v;
    fbk_v = fbk_chassis_v;
    fbk_acc = fbk_chassis_acc;
  }
  // send target acc
  void outputcontrol(float *acc) { *acc = target_acc; }

  /**
   * @brief get trajectory
   */
  void gettracjectory(const nav_msgs::Path &msg) {
    control_msgs::Pathpoint trajectory_point;
    trajectory.clear();
    if (msg.poses.size() == 0)
      ROS_ERROR(" Longitute Controller doesn't get Tracjectory!!!!! ");
    for (int i = 0; i < msg.poses.size(); i++) {
      trajectory_point.x = msg.poses[i].pose.position.x;
      trajectory_point.y = msg.poses[i].pose.position.y;
      trajectory.push_back(trajectory_point);
    }
  }
  /**
   * @brief get_accandjerk_boundary_
   */
  void get_accandjerk_boundary_(float acc_boundary_max_,
                                float acc_boundary_min_,
                                float jerk_boundary_max_,
                                float jerk_boundary_min_) {
    acc_boundary_max = acc_boundary_max_;
    acc_boundary_min = acc_boundary_min_;
    jerk_boundary_max = jerk_boundary_max_;
    jerk_boundary_min = jerk_boundary_min_;
  }
};

}  // namespace Control
#endif