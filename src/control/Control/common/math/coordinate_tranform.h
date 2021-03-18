/*
 * @Descripttion:
 * @version:
 * @Author: xiangzhang
 * @Date: 2020-09-28 13:28:59
 * @LastEditors: xiangzhang
 * @LastEditTime: 2020-10-10 19:29:17
 */
#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/QR>
#include <vector>
using namespace std;

namespace Control {
/**
 * @name:
 * @brief: tranform some pathpoints to coordinate that origin is
 * vehicle_x,vehicle_y
 * @param {type}
 * @return transform_y
 */
std::pair<double, double> pathpointstransformcoordinate(
    float origin_x, float origin_y, float origin_theta,
    const vector<float> &path_x, const vector<float> &path_y);
/**
 * @name:
 * @brief: tranform  pathpoint to coordinate that origin is
 * vehicle_x,vehicle_y
 * @param {type}
 * @return transform_y
 */
std::pair<double, double> pointtransformcoordinate(float origin_x,
                                                   float origin_y,
                                                   float origin_theta,
                                                   float target_x,
                                                   float target_y);
}  // namespace Control