
#ifndef LONCONTROL_H
#define LONCONTROL_H
//*init longitude calss*/

#include <numeric>
#include <queue>
#include <vector>

#include "../common/pid_controller.h"
#include "../message/control_msgs.h"
#include "nav_msgs/Path.h"
#include "ros/ros.h"
//filter
#include "../../common/filter/digital_filter.h"
#include "../../common/filter/digital_filter_coefficients.h"
#include <mutex>
using namespace std;
namespace Control
{

  class LonControl
  {
  private:
    //lock mutex
    std::mutex g_mutex_lon_pose;
    std::mutex g_mutex_lon_chassis;
    std::mutex g_mutex_lon_trajectory;

    /*PIDController*/
    PIDController speed_pid_controller_;
    PIDController position_pid_controller_;
    //FLSAG
    int USE_CHASSIS_ACC_LIMIT_FLAG = 0;
    /* data   */
    float target_v;               // desire velocity
    float fbk_v;                  // chassis's  velocity
    float fbk_acc;                // chassis's  accerlation
    float target_acc;             // desire acc
    bool Pathlength_flag = false; // judge path end

    float speed_error = 0;
    float speed_compenzation_error = 0;
    float target_acc_old = 0;
    float dead_throttle = 0;

    float target_acc_min = -20;
    float target_acc_max = 20;
    float target_acc_step = 0.01;
    vector<float> filterdata;
    float dt = 0.02;
    int cutoff = 0;
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
    vector<control_msgs::Pathpoint> trajectory_rec;
    control_msgs::Pathpoint vehicle;
    control_msgs::Vehicle_status chassis_;

    DigitalFilter digital_filter_pitch_;
    /**
   * @brief calculate speed_error
   * @param target_v(m/s)
   * @param fbk_v(m/s)
   */
    float GetVelocityError(float ref_v, float chassis_v);

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
    float find_min_point(float _preview_x_rec,
                         float _preview_y_rec, int *count_min);

  public:
    // input cycle time
    LonControl();
    ~LonControl();

    /**
   * @brief  longitudinal controller  computer acc_target
   */
    void Loncontrolrun();
    /**
   * @brief reset longitudinal controller
   * @return Status reset status
   */
    bool Reset();
    // vehicle coordinate
    void getvehiclecoordinate(const control_msgs::Pathpoint &vechile_pose)
    {
      lock_guard<mutex> lock(g_mutex_lon_pose);
      vehicle = vechile_pose;
    }
    // chssis datas input km/h
    void inputcontroldata(float target_chassis_v, control_msgs::Vehicle_status &vehicle_status_msg)
    {
      lock_guard<mutex> lock(g_mutex_lon_chassis);
      target_v = target_chassis_v;
      chassis_ = vehicle_status_msg;
    }
    // send target acc
    void outputcontrol(float *acc) { *acc = target_acc; }

    /**
   * @brief get trajectory
   */
    void setWaypoints(const vector<control_msgs::Pathpoint> &msg_path)
    {
      // TODO::update Path
      // input::Path(x,y)
      lock_guard<mutex> lock(g_mutex_lon_trajectory);
      control_msgs::Pathpoint trajectory_point;
      trajectory_rec.clear();
      if (msg_path.size() == 0)
        ROS_ERROR(" Longitute Controller doesn't get Tracjectory!!!!! ");
      for (int i = 1; i < msg_path.size(); i++)
      {
        trajectory_point.x = msg_path[i].x;
        trajectory_point.y = msg_path[i].y;
        trajectory_rec.push_back(trajectory_point);
      }
    }
    /**
   * @brief get_accandjerk_boundary_
   */
    void get_accandjerk_boundary_(float acc_boundary_max_,
                                  float acc_boundary_min_,
                                  float jerk_boundary_max_,
                                  float jerk_boundary_min_)
    {
      acc_boundary_max = acc_boundary_max_;
      acc_boundary_min = acc_boundary_min_;
      jerk_boundary_max = jerk_boundary_max_;
      jerk_boundary_min = jerk_boundary_min_;
    }
  };

} // namespace Control
#endif
