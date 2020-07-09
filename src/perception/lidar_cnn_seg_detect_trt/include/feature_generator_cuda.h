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

// #include <pcl/point_types.h>
// #include <pcl/point_cloud.h>

// #include <vector>

// struct FeatureParameter {

// };

class FeatureGenerator {
  #define LOG_TABLE_SIZE  1024
public:
  FeatureGenerator() {}
  ~FeatureGenerator() {}

  // bool init(std::vector<float>* out_blob);
  bool init(void* out_blob, int height, int width, float range, 
      float min_height, float max_height, bool intensity_normalized, int batchsize, int channel);
  void generate(const void* pointcloud_ptr, int num);
// private:
//   float logCount(int count);
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
  int channel_bytes_size_;
  int dev_out_blob_bytes_;

  // raw feature data in device
  float* dev_max_height_data_;
  float* dev_mean_height_data_;
  float* dev_count_data_;
  float* dev_direction_data_;
  float* dev_top_intensity_data_;
  float* dev_mean_intensity_data_;
  float* dev_distance_data_;
  float* dev_nonempty_data_;

  float* dev_out_blob_; 
  float* dev_log_table_; 

  // cpu 
  // float* out_blob_;
  float* max_height_data_;
  float* top_intensity_data_;
};

#endif //FEATURE_GENERATOR_H
