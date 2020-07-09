/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef FEATURE_GENERATOR_H
#define FEATURE_GENERATOR_H

#include "util.h"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <vector>

// struct FeatureParameter {

// };

class FeatureGenerator {
public:
  FeatureGenerator() {}
  ~FeatureGenerator() {}

  // bool init(std::vector<float>* out_blob);
  bool init(std::vector<float>* out_blob, int height, int width, float range, 
      float min_height, float max_height, bool intensity_normalized, int batchsize, int channel);
  void generate(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_ptr);

private:
  int width_ = 640;
  int height_ = 640;
  int channel_ = 8;
  int batch_ = 1;

  float range_ = 60;
  bool intensity_normalized_ = false;
  float min_height_ = 0.0;
  float max_height_ = 0.0;
  int channel_size_ = 0;
  // raw feature data
  float* max_height_data_ = nullptr;
  float* mean_height_data_ = nullptr;
  float* count_data_ = nullptr;
  float* direction_data_ = nullptr;
  float* top_intensity_data_ = nullptr;
  float* mean_intensity_data_ = nullptr;
  float* distance_data_ = nullptr;
  float* nonempty_data_ = nullptr;

  // output Caffe blob
  std::vector<float>* out_blob_ = nullptr;
  std::vector<float> log_table_;
  // point index in feature map
  std::vector<int> map_idx_;
  float logCount(int count);
};

#endif //FEATURE_GENERATOR_H
