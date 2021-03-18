/*
 * @Descripttion:
 * @version:
 * @Author: xiangzhang
 * @Date: 2020-09-30 15:53:31
 * @LastEditors: xiangzhang
 * @LastEditTime: 2020-10-09 11:25:35
 */
/**
 * @file curve_math.h
 **/

#pragma once
#include <cmath>
#include <utility>
#include <vector>
namespace Control {

class Trajectory_Analyze {
 public:
  //   Trajectory_Analyze() = delete;
  /**
   * @brief Compute the curvature (kappa) given curve X = (x(t), y(t))
   *        which t is an arbitrary parameter.
   * @param dx dx / dt
   * @param d2x d(dx) / dt
   * @param dy dy / dt
   * @param d2y d(dy) / dt
   * @return the curvature
   */
  static float ComputeCurvature(const float dx, const float d2x, const float dy,
                                const float d2y);

  /**
   * @brief Compute the curvature change rate w.r.t. curve length (dkappa) given
   * curve X = (x(t), y(t))
   *        which t is an arbitrary parameter.
   * @param dx dx / dt
   * @param d2x d(dx) / dt
   * @param dy dy / dt
   * @param d2y d(dy) / dt
   * @param d3x d(d2x) / dt
   * @param d3y d(d2y) / dt
   * @return the curvature change rate
   */
  static float ComputeCurvatureDerivative(const float dx, const float d2x,
                                          const float d3x, const float dy,
                                          const float d2y, const float d3y);
  /**
   * @brief Compute the curvature,curvature derivative,distance and yaw
   * east = 0deg,North 90deg
   * @return the curvature complete flag
   */
  bool computePathProfile(const std::vector<std::pair<float, float>>& xy_points,
                          std::vector<float>* headings,
                          std::vector<float>* accumulated_s,
                          std::vector<float>* kappas,
                          std::vector<float>* dkappas);
  /**
   * @brief Compute the Acceleration of trajectory
   * @return the Acceleration complete flag
   */
  bool computeTrajectoryAcceleration(const std::vector<float>& velocity_points,
                                     std::vector<float>* accelerations_points);
  /**
   * @brief Compute the Acceleration and Jerk of trajectory
   * @return the  complete flag
   */
  bool computeTrajectoryAcceleration_Jerk(
      const std::vector<float>& velocity_points,
      const std::vector<std::pair<float, float>>& xy_points,
      std::vector<float>* accelerations_points,
      std::vector<float>* jerks_points);
};

}  // namespace Control
