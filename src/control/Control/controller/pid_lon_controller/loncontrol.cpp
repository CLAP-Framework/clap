// author zx
// date 2020.6.16
// function :longitude control
//
// test

#include "loncontrol.h"

#include <yaml-cpp/yaml.h>

#include <ostream>

#include "ros/ros.h"

using namespace Control;
using namespace std;
#ifndef MAX_PATH
#define MAX_PATH 260
#endif

LonControl::LonControl()
{
  lon_control_init();
}

LonControl::~LonControl() {}

template <typename T>
T getParam(const YAML::Node &node, const string &name, const T &defaultValue)
{
  T v;
  try
  {
    v = node[name].as<T>();
    std::cout << "Found parameter: " << name << ",\tvalue: " << v << std::endl;
  }
  catch (std::exception e)
  {
    v = defaultValue;
    std::cout << "Cannot find parameter: " << name
              << ",\tassigning  default: " << v << std::endl;
  }
  return v;
}

/**Init control param ,use yaml file*/
void LonControl::lon_control_init()
{
  char config_path[MAX_PATH] = {0};

  if (getenv("ZZZ_ROOT") == NULL)
    ROS_ERROR(
        "Not find env (ZZZ_ROOT)!!!!  Please set MINI_AUTO_ROOT={your "
        "mini_auto absolute path},like "
        "export=MINI_AUTO_ROOT=/home/username/");

  strcpy(config_path, getenv("ZZZ_ROOT"));
  strcat(config_path, "/zzz/src/control/Control/conf/loncontroller_pid.yaml");
  YAML::Node dset_config = YAML::LoadFile(config_path);

  // YAML::Node dset_config = YAML::LoadFile(
  //     "/home/zx/2.control_gitlab/src/Control/conf/loncontroller_pid.yaml");
  YAML::Node params = dset_config["_xpg3_pid_parameters"];

  /****VehicleModle*****/
  Kp = getParam<float>(params, "Kp", 0);
  Ki = getParam<float>(params, "Ki", 0);
  Kd = getParam<float>(params, "Kd", 0);
  target_acc_smoothnumber = getParam<float>(params, "target_acc_smoothnumber", 0);
  speed_error_min = getParam<float>(params, "speed_error_min", 0);
  speed_error_max = getParam<float>(params, "speed_error_max", 0);

  dt = getParam<float>(params, "dt", 0);
  integrator_saturation_max = getParam<float>(params, "integrator_saturation_max", 0);
  integrator_saturation_min = getParam<float>(params, "integrator_saturation_min", 0);

  out_acc_max = getParam<float>(params, "out_acc_max", 0);
  out_acc_min = getParam<float>(params, "out_acc_min", 0);
  out_acc_rate_max = getParam<float>(params, "out_acc_rate_max", 0);
  out_acc_rate_min = getParam<float>(params, "out_acc_rate_min", 0);

  speed_pid_controller_.Init(integrator_saturation_max, integrator_saturation_min);
  //filter
  cutoff = getParam<float>(params, "cutoff", 0);

  position_pid_controller_.Init(0, 0);

  //filter
  // Low pass filter
  std::vector<double> den(3, 0.0);
  std::vector<double> num(3, 0.0);
  LpfCoefficients(dt, cutoff, &den, &num);
  digital_filter_pitch_.set_coefficients(den, num);
  /////////
  USE_CHASSIS_ACC_LIMIT_FLAG = getParam<float>(params, "USE_CHASSIS_ACC_LIMIT_FLAG", 0);
}

bool LonControl::Reset()
{
  speed_pid_controller_.Reset();
  position_pid_controller_.Reset();
  return true;
}
////////////////////////////////////////////////////////////////////////////////////////////
// speed
float LonControl::GetVelocityError( float ref_v,  float chassis_v)
{
  float speed_error;
  speed_error = target_v - chassis_v;
  if (speed_error < speed_error_min)
    speed_error = speed_error_min;
  if (speed_error > speed_error_max)
    speed_error = speed_error_max;

  return speed_error;
}

float LonControl::VelocityControl(float speed_control_error)
{
  static float control_out = 0;
  bool integratorhold_flag = false;
  int integratorSaturation_flag = 0;
  speed_pid_controller_.SetPID(Kp, Ki, Kd);
  control_out = speed_pid_controller_.Control(speed_control_error, dt);
  integratorhold_flag = speed_pid_controller_.IntegratorHold();
  integratorSaturation_flag =
      speed_pid_controller_.IntegratorSaturationStatus();
  speed_pid_controller_.SetIntegratorHold(0);
  return control_out;
}

//////////////////////////////////////////////////////////////////////////////////////
// main
void LonControl::Loncontrolrun()
{
  trajectory = trajectory_rec;
  int trajectory_size = trajectory.size();
  if (trajectory_size >= 1)
  {
    int nextpoint_num = 0;
    // find_min_point(vehicle.x, vehicle.y, &nextpoint_num);
    // position_pid_controller_.SetPID(0.3, 0, 0);
    // double position_error = sqrt(pow(trajectory[trajectory_size - 1].x - trajectory[nextpoint_num].x, 2) + pow(trajectory[trajectory_size - 1].y - trajectory[nextpoint_num].y, 2));

    // double speed_compensition = position_pid_controller_.Control(position_error, dt);
    // if (speed_compensition > 0.5)
    //   speed_compensition = 0.5; //m/s
    // position_pid_controller_.IntegratorHold();
    // position_pid_controller_.IntegratorSaturationStatus();
    // position_pid_controller_.SetIntegratorHold(0);
    // cout<<" position_error "<<position_error<<"  "<<nextpoint_num<<endl;
    // if (position_error < 3)
    // {
    //   target_v = speed_compensition;
    // }
    // if (target_v < 0.3)
    // {
    //   target_v = 0.3;
    // }
    speed_error = GetVelocityError(target_v, chassis_.velocity_fd * 3.6);
    target_acc = VelocityControl(speed_error);
    target_acc = smoothfilter(target_acc, target_acc_smoothnumber);
    //add pitch gain
    double pitch_filter = digital_filter_pitch_.Filter(vehicle.pitch);
    double picth_acc = sin(pitch_filter);
    if (chassis_.gear_fd == 2) //R gear
    {
      picth_acc = -picth_acc;
    }
    // target_acc = limit_param(target_acc, target_acc_old, -2, 2, -5, 5, 0.05);
    //stoppoint
    // if (trajectory[nextpoint_num - 1].s > 0.2 && trajectory[nextpoint_num - 1].s != 0 && position_error > 1 && trajectory_size != nextpoint_num)
    // {
    //   target_acc = -1.2 * (chassis_.velocity_fd * chassis_.velocity_fd) / (2 * (trajectory[trajectory.size() - 1].s - trajectory[nextpoint_num].s));
    // }
    // target_acc = target_acc + picth_acc;
    // if (position_error < 0.15 || trajectory_size == nextpoint_num)
    // {
    //   target_acc = -2;
    // }
    //
    if (USE_CHASSIS_ACC_LIMIT_FLAG)
    {
      target_acc_old = chassis_.acceleration_fd;
    }
    target_acc = limit_param(target_acc, target_acc_old, out_acc_min, out_acc_max,
                             out_acc_rate_min, out_acc_rate_max, dt);
    target_acc_old = target_acc;
  }
  else
  {
    target_acc = 0;
    target_acc_old = target_acc;
  }
}

float LonControl::find_min_point(float _preview_x_tmp,
                                 float _preview_y_tmp, int *count_min)
{
  // TODO::find closest point error (left - right)
  // input:: coordinate (m,m)
  // outpu:: the index of closest point
  float _L_min = 20.0;
  float _L;
  float _x_min = -1.0;
  float _y_min = -1.0;
  float _Oss_tmp = 0;
  int _count_min = 0;
  for (int _path_ii = 0; _path_ii < trajectory.size(); _path_ii++)
  {
    _L = sqrt(pow((_preview_x_tmp - trajectory[_path_ii].x), 2) +
              pow((_preview_y_tmp - trajectory[_path_ii].y), 2));
              // cout<<"trajectory[_path_ii].x  "<<trajectory[_path_ii].x<<"  "<<_L_min<<endl; 
    if (_L <= _L_min)
    {
      _L_min = _L;
      _x_min = trajectory[_path_ii].x;
      _y_min = trajectory[_path_ii].y;
      _count_min = _path_ii;
    }
    else
    {
      _L_min = _L_min;
      _x_min = _x_min;
      _y_min = _y_min;
       _count_min= _count_min;
    }
  }
  *count_min = _count_min;
}
// TODO::replace filer.h
float LonControl::smoothfilter(float param, int datanumber)
{
  if (filterdata.size() >= datanumber)
  {
    for (int i = 0; i < datanumber; i++)
    {
      filterdata[i] = filterdata[i + 1];
      // cout<<"  filterdata[i] "<< filterdata[i]<<endl;
    }
    filterdata[datanumber] = param;
    // cout<<"  filterdata[i] "<< param<<endl;
  }
  else
  {
    filterdata.push_back(param);
  }
  double sum =
      std::accumulate(std::begin(filterdata), std::end(filterdata), 0.0);
  double mean = sum / filterdata.size();
  return mean;
}

// TODO::replace filer.h
float LonControl::limit_param(float param, float param_last, float param_min,
                              float param_max, float param_min_rate,
                              float param_max_rate, float dt)
{
  float d_param = param - param_last;
  if ((d_param >= 0) && (d_param > param_max_rate * dt))
  {
    d_param = param_max_rate * dt;
    param = param_last + d_param;
  }
  else
  {
    if ((d_param < 0) && (abs(d_param) > abs(param_min_rate * dt)))
    {
      d_param = param_min_rate * dt;
      param = param_last + d_param;
    }
    else
    {
      param = param;
    }
  }
  if (param > param_max)
    param = param_max;
  if (param < param_min)
    param = param_min;
  return param;
}
