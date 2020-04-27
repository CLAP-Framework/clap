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

#include "feature_generator.h"
#include <memory.h>
#include <cmath>

// headers in CUDA
#include <cuda_runtime_api.h>

// #include <pcl/point_types.h>
// #include <pcl/point_cloud.h>

//using MACRO to allocate memory inside CUDA kernel
#define NUM_3D_BOX_CORNERS_MACRO 8
#define NUM_2D_BOX_CORNERS_MACRO 4
#define NUM_THREADS_MACRO 64 // need to be changed when NUM_THREADS is changed

#define DIVUP(m, n) ((m) / (n) + ((m) % (n) > 0))

#define GPU_CHECK(ans)                                                                                                 \
  {                                                                                                                    \
    GPUAssert((ans), __FILE__, __LINE__);                                                                              \
  }
inline void GPUAssert(cudaError_t code, const char* file, int line, bool abort = true)
{
  if (code != cudaSuccess)
  {
    fprintf(stderr, "GPUassert: %s %s %d\n", cudaGetErrorString(code), file, line);
    if (abort)
      exit(code);
  }
}

bool FeatureGenerator::init(float* data,
    const int width,
    const int height,
    const int channel,
    const int batch,
    const float range,
    const float min_height,
    const float max_height,
    const int intensity_normalized) : 
        data_(data),
        width_(width),
        height_(height),
        channel_(channel),
        batch_(batch),
        range_(range),
        min_height_(min_height),
        max_height_(max_height),
        intensity_normalized_(intensity_normalized) {
    
    // set output blob and log lookup table
    GPU_CHECK(cudaMallocManage((void**)&data, batch_*channel_*height_*width_));

    // table for log(x+1) 
    log_table_.resize(256);
    for (size_t i = 0; i < log_table_.size(); ++i) {
        log_table_[i] = std::log1p(static_cast<float>(i));
    }

    float* out_blob_data = data_;
    int channel_index = 0;
    channel_size_  = height_ * width_;
    max_height_data_     = out_blob_data + channel_size_ * channel_index++;
    mean_height_data_    = out_blob_data + channel_size_ * channel_index++;
    count_data_          = out_blob_data + channel_size_ * channel_index++;
    direction_data_      = out_blob_data + channel_size_ * channel_index++;
    top_intensity_data_  = out_blob_data + channel_size_ * channel_index++;
    mean_intensity_data_ = out_blob_data + channel_size_ * channel_index++;
    distance_data_       = out_blob_data + channel_size_ * channel_index++;
    nonempty_data_       = out_blob_data + channel_size_ * channel_index++;

    constexpr double K_CV_PI = 3.1415926535897932384626433832795;
    for (int row = 0; row < height_; ++row) {
        for (int col = 0; col < width_; ++col) {
            int idx = row * width_ + col;
            // * row <-> x, column <-> y
            float center_x = Pixel2Pc(row, height_, range_);
            float center_y = Pixel2Pc(col, width_, range_);
            
            direction_data_[idx] =
                static_cast<float>(std::atan2(center_y, center_x) / (2.0 * K_CV_PI));
            distance_data_[idx] =
                static_cast<float>(std::hypot(center_x, center_y) / 60.0 - 0.5);
        }
    }
    return true;
}

float FeatureGenerator::logCount(int count) {
    if (count < static_cast<int>(log_table_.size())) {
        return log_table_[count];
    }
    return std::log(static_cast<float>(1 + count));
}

void FeatureGenerator::generate( const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_ptr) {
    const auto& points = pc_ptr->points;
    

    int siz = height_ * width_;
    size_t byte_size = channel_size_*sizeof(float);
    // memset((void*)max_height_data_, 0, byte_size);
    // max_height_data_ need to be reset to min_height_
    for (int i=0; i<channel_size_; i++) {
        max_height_data_[i] = min_height_;
    }
    memset((void*)mean_height_data_, 0, byte_size);
    memset((void*)count_data_, 0, byte_size);
    // direction_data_ don't need to be reset 
    memset((void*)top_intensity_data_, 0, byte_size);
    memset((void*)mean_intensity_data_, 0, byte_size);
    // distance_data_  don't need to be reset 
    memset((void*)nonempty_data_, 0, byte_size);


    map_idx_.resize(points.size());
    float inv_res_x = 0.5 * static_cast<float>(width_) / static_cast<float>(range_);
    float inv_res_y = 0.5 * static_cast<float>(height_) / static_cast<float>(range_);

    for (size_t i = 0; i < points.size(); ++i) {
        if (points[i].z <= min_height_ || points[i].z >= max_height_) {
            map_idx_[i] = -1;
            continue;
        }
        // * the coordinates of x and y are exchanged here
        // (row <-> x, column <-> y)
        int pos_x = F2I(points[i].y, range_, inv_res_x);  // col
        int pos_y = F2I(points[i].x, range_, inv_res_y);  // row
        if (pos_x >= width_ || pos_x < 0 || pos_y >= height_ || pos_y < 0) {
            map_idx_[i] = -1;
            continue;
        }
        map_idx_[i] = pos_y * width_ + pos_x;

        int idx = map_idx_[i];
        float pz = points[i].z;
        float pi;
        if (intensity_normalized_) {
            pi = points[i].intensity;
        } else {
            pi = points[i].intensity / 255.0;  
        }
        
        if (max_height_data_[idx] < pz) {
            max_height_data_[idx] = pz;
            top_intensity_data_[idx] = pi;
        }

        mean_height_data_[idx] += static_cast<float>(pz);
        mean_intensity_data_[idx] += static_cast<float>(pi);
        count_data_[idx] += float(1);
    }
    constexpr double EPS = 1e-6;
    for (int i = 0; i < channel_size_; ++i) {
        if (count_data_[i] < EPS) {
            // TODO: 
            max_height_data_[i] = float(0);
        } else {
            mean_height_data_[i] /= count_data_[i];
            mean_intensity_data_[i] /= count_data_[i];
            nonempty_data_[i] = float(1);
        }
        count_data_[i] = logCount(static_cast<int>(count_data_[i]));
    }
}
