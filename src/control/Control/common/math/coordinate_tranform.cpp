/*
 * @Descripttion:
 * @version:
 * @Author: xiangzhang
 * @Date: 2020-09-28 13:28:55
 * @LastEditors: xiangzhang
 * @LastEditTime: 2020-10-10 19:34:28
 */

#include "coordinate_tranform.h"

namespace Control {

std::pair<double, double> pathpointstransformcoordinate(
    float origin_x, float origin_y, float origin_theta,
    const vector<float> &path_x, const vector<float> &path_y) {
  //
  //逆时针旋转theta(0~360) 为正
  //车身坐标系为右手坐标系
  std::pair<double, double> point;
  Eigen::Matrix2d transform;
  vector<float> transform_path_x;
  vector<float> transform_path_y;
  float transform_x = 0;
  float transform_y = 0;
  transform_path_x.clear();
  transform_path_y.clear();
  //左右手坐标系不一样?
  transform << cos(origin_theta), -sin(origin_theta), sin(origin_theta),
      cos(origin_theta);

  for (int i = 0; i < path_x.size(); i++) {
    transform_x = (path_x[i] - origin_x) * cos(origin_theta) +
                  sin(origin_theta) * (path_y[i] - origin_y);
    transform_y = (path_x[i] - origin_x) * -1 * sin(origin_theta) +
                  cos(origin_theta) * (path_y[i] - origin_y);
    transform_x = abs(transform_x);
    transform_path_x.push_back(transform_x);
    transform_path_y.push_back(transform_y);
  }
  auto smallest = std::min_element(std::begin(transform_path_x),
                                   std::end(transform_path_x));
  int smallest_index = std::distance(std::begin(transform_path_x), smallest);
  point.first = transform_path_x[smallest_index];
  point.second = transform_path_y[smallest_index];
  return point;
}

std::pair<double, double> pointtransformcoordinate(float origin_x,
                                                   float origin_y,
                                                   float origin_theta,
                                                   float target_x,
                                                   float target_y) {
  //
  std::pair<double, double> point;
  Eigen::Matrix2d transform;

  transform << cos(origin_theta), -sin(origin_theta), sin(origin_theta),
      cos(origin_theta);

  point.first = (target_x - origin_x) * cos(origin_theta) +
                sin(origin_theta) * (target_y - origin_y);
  point.second = (target_x - origin_x) * -1 * sin(origin_theta) +
                 cos(origin_theta) * (target_y - origin_y);

  return point;
}
}  // namespace Control