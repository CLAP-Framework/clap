/*
 * @Descripttion:
 * @version:
 * @Author: xiangzhang
 * @Date: 2020-09-01 14:13:52
 * @LastEditors: xiangzhang
 * @LastEditTime: 2020-10-10 20:51:05
 */

#include "limitfilter.h"

namespace Control {
double limitparamrate(float param, float param_lasttime, float param_min,
                      float param_max, float param_step) {
  if ((param - param_lasttime) > param_step)
    param = param_lasttime + param_step;
  else if ((param - param_lasttime) < -param_step)
    param = param_lasttime - param_step;
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

double limitparamvalue(float param, float param_min, float param_max) {
  if (param > param_max) param = param_max;
  if (param < param_min) param = param_min;
  return param;
}
double limit_param(float param, float param_lasttime, float param_min,
                   float param_max, float param_max_rate, float param_min_rate,
                   float dt) {
  float d_param = param - param_lasttime;

  if ((d_param >= 0) and (d_param > param_max_rate * dt)) {
    d_param = param_max_rate * dt;
    param = param_lasttime + d_param;
  } else {
    if ((d_param < 0) && (abs(d_param) > abs(param_min_rate * dt))) {
      d_param = param_min_rate * dt;
      param = param_lasttime + d_param;
    } else
      param = param;
    if (param > param_max) param = param_max;
    if (param < param_min) param = param_min;
  }

  return param;
}
}  // namespace Control