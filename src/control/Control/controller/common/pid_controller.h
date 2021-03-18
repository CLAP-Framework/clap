/*
 * @Descripttion:
 * @version:
 * @Author: xiangzhang
 * @Date: 2020-09-10 17:29:02
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2020-09-15 15:25:39
 * @FilePath: /ccc/src/review_code/controller/common/pid_controller.h
 */

/**
 * @file pid_controller.h
 * @brief Defines the PIDController class
 */

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H
#include <iostream>
class PIDController {
 private:
  /* data */
  double kp_ = 0.0;
  double ki_ = 0.0;
  double kd_ = 0.0;
  double kaw_ = 0.0;
  double previous_error_ = 0.0;
  double previous_output_ = 0.0;
  double integral_ = 0.0;
  double integrator_saturation_high_ = 1;
  double integrator_saturation_low_ = -1;
  bool first_hit_ = false;
  bool integrator_enabled_ = false;
  bool integrator_hold_ = false;
  int integrator_saturation_status_ = 0;
  // Only used for pid_BC_controller and pid_IC_controller
  double output_saturation_high_ = 0.0;
  double output_saturation_low_ = 0.0;
  int output_saturation_status_ = 0;

 public:
  /**
   *@brief Init PID parameter
   */
  void Init(float integrator_saturation_max, float integrator_saturation_min);
  void SetPID(float kp, float ki, float kd);
  void Reset();
  double Control(const double error, const double dt);
  int IntegratorSaturationStatus();
  bool IntegratorHold();
  void SetIntegratorHold(bool hold);

  PIDController(/* args */);
  ~PIDController();
};

#endif