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

LonControl::LonControl() { lon_control_init(); }

LonControl::~LonControl() {}

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

/**Init control param ,use yaml file*/
void LonControl::lon_control_init() {
  char config_path[MAX_PATH] = {0};

  if (getenv("ZZZ_ROOT") == NULL)
    ROS_ERROR(
        "Not find env (ZZZ_ROOT)!!!!  Please set MINI_AUTO_ROOT={your "
        "mini_auto absolute path},like "
        "export=MINI_AUTO_ROOT=/home/username/");

  strcpy(config_path, getenv("ZZZ_ROOT"));
  strcat(config_path,
         "/zzz/src/control/icvfollower/config/xpg3_pid_velocity.yaml");

  YAML::Node dset_config = YAML::LoadFile(config_path);

  YAML::Node PID_parameters = dset_config["_xpg3_pid_parameters"];

  /****VehicleModle*****/
  Kp = getParam<float>(PID_parameters, "Kp", 0);
  Ki = getParam<float>(PID_parameters, "Ki", 0);
  Kd = getParam<float>(PID_parameters, "Kd", 0);
  target_acc_smoothnumber =
      getParam<float>(PID_parameters, "target_acc_smoothnumber", 0);
  speed_error_min = getParam<float>(PID_parameters, "speed_error_min", 0);
  speed_error_max = getParam<float>(PID_parameters, "speed_error_max", 0);

  dt = getParam<float>(PID_parameters, "dt", 0);
  integrator_saturation_max =
      getParam<float>(PID_parameters, "integrator_saturation_max", 0);
  integrator_saturation_min =
      getParam<float>(PID_parameters, "integrator_saturation_min", 0);

  out_acc_max = getParam<float>(PID_parameters, "out_acc_max", 0);
  out_acc_min = getParam<float>(PID_parameters, "out_acc_min", 0);
  out_acc_rate_max = getParam<float>(PID_parameters, "out_acc_rate_max", 0);
  out_acc_rate_min = getParam<float>(PID_parameters, "out_acc_rate_min", 0);
  speed_pid_controller_.Init(integrator_saturation_max,
                             integrator_saturation_min);
}
float LonControl::GetVelocityError(const float ref_v, const float chassis_v) {
  float speed_error;
  speed_error = target_v - fbk_v;
  if (speed_error < speed_error_min) speed_error = speed_error_min;
  if (speed_error > speed_error_max) speed_error = speed_error_max;

  return speed_error;
}
float LonControl::VelocityControl(float speed_control_error) {
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

void LonControl::Loncontrolrun() {
  speed_error = GetVelocityError(target_v, fbk_v);
  target_acc = VelocityControl(speed_error);
  target_acc = smoothfilter(target_acc, target_acc_smoothnumber);
  // target_acc = limit_param(target_acc, target_acc_old, -2, 2, -5, 5, 0.05);

  target_acc = limit_param(target_acc, target_acc_old, out_acc_min, out_acc_max,
                           out_acc_rate_min, out_acc_rate_max, dt);
  target_acc_old = target_acc;
}

float LonControl::smoothfilter(float param, int datanumber) {
  if (filterdata.size() >= datanumber) {
    for (int i = 0; i < datanumber; i++) {
      filterdata[i] = filterdata[i + 1];
      // cout<<"  filterdata[i] "<< filterdata[i]<<endl;
    }
    filterdata[datanumber] = param;
    // cout<<"  filterdata[i] "<< param<<endl;
  } else {
    filterdata.push_back(param);
  }
  double sum =
      std::accumulate(std::begin(filterdata), std::end(filterdata), 0.0);
  double mean = sum / filterdata.size();
  return mean;
}
float LonControl::limit_param(float param, float param_last, float param_min,
                              float param_max, float param_min_rate,
                              float param_max_rate, float dt) {
  float d_param = param - param_last;
  if ((d_param >= 0) && (d_param > param_max_rate * dt)) {
    d_param = param_max_rate * dt;
    param = param_last + d_param;
  } else {
    if ((d_param < 0) && (abs(d_param) > abs(param_min_rate * dt))) {
      d_param = param_min_rate * dt;
      param = param_last + d_param;
    } else {
      param = param;
    }
  }
  if (param > param_max) param = param_max;
  if (param < param_min) param = param_min;
  return param;
}