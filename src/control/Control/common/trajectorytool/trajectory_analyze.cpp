/*
 * @Descripttion:
 * @version:
 * @Author: xiangzhang
 * @Date: 2020-09-30 15:53:31
 * @LastEditors: xiangzhang
 * @LastEditTime: 2020-10-09 12:00:44
 */

/**
 * @file curve_math.cc
 **/

#include "trajectory_analyze.h"

namespace Control {

// kappa = (dx * d2y - dy * d2x) / [(dx * dx + dy * dy)^(3/2)]
float Trajectory_Analyze::ComputeCurvature(const float dx, const float d2x,
                                           const float dy, const float d2y) {
  const float a = dx * d2y - dy * d2x;
  auto norm_square = dx * dx + dy * dy;
  auto norm = std::sqrt(norm_square);
  const float b = norm * norm_square;
  return a / b;
}

float Trajectory_Analyze::ComputeCurvatureDerivative(
    const float dx, const float d2x, const float d3x, const float dy,
    const float d2y, const float d3y) {
  const float a = dx * d2y - dy * d2x;
  const float b = dx * d3y - dy * d3x;
  const float c = dx * d2x + dy * d2y;
  const float d = dx * dx + dy * dy;

  return (b * d - 3.0 * a * c) / (d * d * d);
}

bool Trajectory_Analyze::computePathProfile(
    const std::vector<std::pair<float, float>>& xy_points,
    std::vector<float>* headings, std::vector<float>* accumulated_s,
    std::vector<float>* kappas, std::vector<float>* dkappas) {
  if (headings->size() != 0) headings->clear();
  if (kappas->size() != 0) kappas->clear();
  if (dkappas->size() != 0) dkappas->clear();

  if (xy_points.size() < 2) {
    return false;
  }
  std::vector<float> dxs;
  std::vector<float> dys;
  std::vector<float> y_over_s_first_derivatives;
  std::vector<float> x_over_s_first_derivatives;
  std::vector<float> y_over_s_second_derivatives;
  std::vector<float> x_over_s_second_derivatives;

  // Get finite difference approximated dx and dy for heading and kappa
  // calculation
  std::size_t points_size = xy_points.size();
  for (std::size_t i = 0; i < points_size; ++i) {
    float x_delta = 0.0;
    float y_delta = 0.0;
    if (i == 0) {
      x_delta = (xy_points[i + 1].first - xy_points[i].first);
      y_delta = (xy_points[i + 1].second - xy_points[i].second);
    } else if (i == points_size - 1) {
      x_delta = (xy_points[i].first - xy_points[i - 1].first);
      y_delta = (xy_points[i].second - xy_points[i - 1].second);
    } else {
      x_delta = 0.5 * (xy_points[i + 1].first - xy_points[i - 1].first);
      y_delta = 0.5 * (xy_points[i + 1].second - xy_points[i - 1].second);
    }
    dxs.push_back(x_delta);
    dys.push_back(y_delta);
  }

  // Heading calculation
  for (std::size_t i = 0; i < points_size; ++i) {
    headings->push_back(std::atan2(dys[i], dxs[i]));
  }

  // Get linear interpolated s for dkappa calculation
  float distance = 0.0;
  accumulated_s->push_back(distance);
  float fx = xy_points[0].first;
  float fy = xy_points[0].second;
  float nx = 0.0;
  float ny = 0.0;
  for (std::size_t i = 1; i < points_size; ++i) {
    nx = xy_points[i].first;
    ny = xy_points[i].second;
    float end_segment_s =
        std::sqrt((fx - nx) * (fx - nx) + (fy - ny) * (fy - ny));
    accumulated_s->push_back(end_segment_s + distance);
    distance += end_segment_s;
    fx = nx;
    fy = ny;
  }

  // Get finite difference approximated first derivative of y and x respective
  // to s for kappa calculation
  for (std::size_t i = 0; i < points_size; ++i) {
    float xds = 0.0;
    float yds = 0.0;
    if (i == 0) {
      xds = (xy_points[i + 1].first - xy_points[i].first) /
            (accumulated_s->at(i + 1) - accumulated_s->at(i));
      yds = (xy_points[i + 1].second - xy_points[i].second) /
            (accumulated_s->at(i + 1) - accumulated_s->at(i));
    } else if (i == points_size - 1) {
      xds = (xy_points[i].first - xy_points[i - 1].first) /
            (accumulated_s->at(i) - accumulated_s->at(i - 1));
      yds = (xy_points[i].second - xy_points[i - 1].second) /
            (accumulated_s->at(i) - accumulated_s->at(i - 1));
    } else {
      xds = (xy_points[i + 1].first - xy_points[i - 1].first) /
            (accumulated_s->at(i + 1) - accumulated_s->at(i - 1));
      yds = (xy_points[i + 1].second - xy_points[i - 1].second) /
            (accumulated_s->at(i + 1) - accumulated_s->at(i - 1));
    }
    x_over_s_first_derivatives.push_back(xds);
    y_over_s_first_derivatives.push_back(yds);
  }

  // Get finite difference approximated second derivative of y and x respective
  // to s for kappa calculation
  for (std::size_t i = 0; i < points_size; ++i) {
    float xdds = 0.0;
    float ydds = 0.0;
    if (i == 0) {
      xdds =
          (x_over_s_first_derivatives[i + 1] - x_over_s_first_derivatives[i]) /
          (accumulated_s->at(i + 1) - accumulated_s->at(i));
      ydds =
          (y_over_s_first_derivatives[i + 1] - y_over_s_first_derivatives[i]) /
          (accumulated_s->at(i + 1) - accumulated_s->at(i));
    } else if (i == points_size - 1) {
      xdds =
          (x_over_s_first_derivatives[i] - x_over_s_first_derivatives[i - 1]) /
          (accumulated_s->at(i) - accumulated_s->at(i - 1));
      ydds =
          (y_over_s_first_derivatives[i] - y_over_s_first_derivatives[i - 1]) /
          (accumulated_s->at(i) - accumulated_s->at(i - 1));
    } else {
      xdds = (x_over_s_first_derivatives[i + 1] -
              x_over_s_first_derivatives[i - 1]) /
             (accumulated_s->at(i + 1) - accumulated_s->at(i - 1));
      ydds = (y_over_s_first_derivatives[i + 1] -
              y_over_s_first_derivatives[i - 1]) /
             (accumulated_s->at(i + 1) - accumulated_s->at(i - 1));
    }
    x_over_s_second_derivatives.push_back(xdds);
    y_over_s_second_derivatives.push_back(ydds);
  }

  for (std::size_t i = 0; i < points_size; ++i) {
    float xds = x_over_s_first_derivatives[i];
    float yds = y_over_s_first_derivatives[i];
    float xdds = x_over_s_second_derivatives[i];
    float ydds = y_over_s_second_derivatives[i];
    float kappa =
        (xds * ydds - yds * xdds) /
        (std::sqrt(xds * xds + yds * yds) * (xds * xds + yds * yds) + 1e-6);
    kappas->push_back(kappa);
  }

  // Dkappa calculation
  for (std::size_t i = 0; i < points_size; ++i) {
    float dkappa = 0.0;
    if (i == 0) {
      dkappa = (kappas->at(i + 1) - kappas->at(i)) /
               (accumulated_s->at(i + 1) - accumulated_s->at(i));
    } else if (i == points_size - 1) {
      dkappa = (kappas->at(i) - kappas->at(i - 1)) /
               (accumulated_s->at(i) - accumulated_s->at(i - 1));
    } else {
      dkappa = (kappas->at(i + 1) - kappas->at(i - 1)) /
               (accumulated_s->at(i + 1) - accumulated_s->at(i - 1));
    }
    dkappas->push_back(dkappa);
  }
  return true;
}

bool computeTrajectoryAcceleration(
    const std::vector<float>& velocity_points,
    const std::vector<std::pair<float, float>>& xy_points,
    std::vector<float>* accelerations_points) {
  if (accelerations_points->size() != 0) accelerations_points->clear();
  if (velocity_points.size() > 2 && xy_points.size() > 2) {
    float fx = xy_points[0].first;
    float fy = xy_points[0].second;
    float nx = 0.0;
    float ny = 0.0;
    accelerations_points->push_back(0);
    for (int i = 1; i < velocity_points.size(); ++i) {
      nx = xy_points[i].first;
      ny = xy_points[i].second;
      float distance = std::sqrt((fx - nx) * (fx - nx) + (fy - ny) * (fy - ny));
      fx = nx;
      fy = ny;
      // v^2 - v0^2 = 2ax
      float acc_d =
          (velocity_points[i] - velocity_points[i - 1]) / (2 * distance);

      accelerations_points->push_back(acc_d);
    }
  }
}

bool computeTrajectoryAcceleration_Jerk(
    const std::vector<float>& velocity_points,
    const std::vector<std::pair<float, float>>& xy_points,
    std::vector<float>* accelerations_points,
    std::vector<float>* jerks_points) {
  // get acceleration from trajectory
  if (accelerations_points->size() != 0) accelerations_points->clear();
  if (velocity_points.size() > 2 && xy_points.size() > 2) {
    float fx = xy_points[0].first;
    float fy = xy_points[0].second;
    float nx = 0.0;
    float ny = 0.0;
    accelerations_points->push_back(0);
    for (int i = 1; i < velocity_points.size(); ++i) {
      nx = xy_points[i].first;
      ny = xy_points[i].second;
      float distance = std::sqrt((fx - nx) * (fx - nx) + (fy - ny) * (fy - ny));
      fx = nx;
      fy = ny;
      // v^2 - v0^2 = 2ax
      float acc_d =
          (velocity_points[i] - velocity_points[i - 1]) / (2 * distance);

      accelerations_points->push_back(acc_d);
      // get jerk
    }
  }
}

}  // namespace Control
