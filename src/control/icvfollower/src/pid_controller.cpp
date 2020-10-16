/*
 * @Author: your name
 * @Date: 1970-01-01 08:00:00
 * @LastEditTime: 2020-09-16 17:38:26
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /icvfollower/src/pid_controller.cpp
 */
#include "pid_controller.h"

#include <cmath>

using namespace std;
PIDController::PIDController(/* args */) {}

PIDController::~PIDController() {}
void PIDController::Init(float integrator_saturation_max,
                         float integrator_saturation_min) {
  previous_error_ = 0.0;
  previous_output_ = 0.0;
  integral_ = 0.0;
  first_hit_ = true;
  integrator_enabled_ = 1;

  integrator_saturation_high_ = integrator_saturation_max;
  integrator_saturation_low_ = integrator_saturation_min;
  integrator_saturation_status_ = 0;
  integrator_hold_ = false;
  output_saturation_high_ = 0;
  output_saturation_low_ = 0;
  output_saturation_status_ = 0;
}
void PIDController::SetPID(float kp, float ki, float kd) {
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
}
void PIDController::Reset() {
  previous_error_ = 0.0;
  previous_output_ = 0.0;
  integral_ = 0.0;
  first_hit_ = true;
  integrator_saturation_status_ = 0;
  output_saturation_status_ = 0;
}
double PIDController::Control(const double error, const double dt) {
  if (dt <= 0) {
    return previous_output_;
  }
  double diff = 0;
  double output = 0;

  if (first_hit_) {
    first_hit_ = false;
  } else {
    diff = (error - previous_error_) / dt;
  }
  // integral hold
  if (!integrator_enabled_) {
    integral_ = 0;
  } else if (!integrator_hold_) {
    integral_ += error * dt * ki_;
    // apply Ki before integrating to avoid steps when change Ki at steady state
    if (integral_ > integrator_saturation_high_) {
      integral_ = integrator_saturation_high_;
      integrator_saturation_status_ = 1;
    } else if (integral_ < integrator_saturation_low_) {
      integral_ = integrator_saturation_low_;
      integrator_saturation_status_ = -1;
    } else {
      integrator_saturation_status_ = 0;
    }
  }

  previous_error_ = error;
  output = error * kp_ + integral_ + diff * kd_;  // Ki already applied
  // cout << "  error * kp_ " << error * kp_ << "integral_ " << integral_
  //      << "  diff * kd_ " << diff * kd_ << endl;
  previous_output_ = output;
  return output;
}
int PIDController::IntegratorSaturationStatus() {
  return integrator_saturation_status_;
}
bool PIDController::IntegratorHold() { return integrator_hold_; }
void PIDController::SetIntegratorHold(bool hold) { integrator_hold_ = hold; }