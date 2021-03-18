/*
 * @Descripttion:save some limit function
 * @version:
 * @Author: xiangzhang
 * @Date: 2020-09-01 13:18:22
 * @LastEditors: xiangzhang
 * @LastEditTime: 2020-10-26 16:03:42
 */

#ifndef LIMITFILTER_H
#define LIMITFILTER_H

#include <cmath>

namespace Control {

/**
 * @name:
 * @test:
 * @function:Limit the step size(Increase and decrease the same step size),
 * maximum and minimum of parameters
 * @msg:
 * @param
 * @return parma that is limited
 */
double limitparamrate(float param, float param_lasttime, float param_min,
                      float param_max, float param_step);
/**
 * @name:
 * @test:
 * @function:limit the maximum and minimum of parameters
 * @msg:
 * @param
 * @return
 */
double limitparamvalue(float param, float param_min, float param_max);
/**
 * @name:
 * @test:
 * @function:Limit the step size(Increase and decrease the different step size),
 * maximum and minimum of parameters
 * @msg:
 * @param {type}
 * @return {type}
 */
double limitparam(float param, float param_lasttime, float param_min,
                  float param_max, float param_max_rate, float param_min_rate,
                  float dt);

}  // namespace Control
#endif
