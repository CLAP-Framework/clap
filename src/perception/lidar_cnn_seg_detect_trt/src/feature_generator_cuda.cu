/*
 * Copyright 2020-2022 Novauto. All rights reserved.
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


#include "feature_generator_cuda.h"
#include "prekernel.h"

#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <memory.h>
#include <cmath>

// #include <pcl/point_types.h>
// #include <pcl/point_cloud.h>
#include <stdio.h>
#include <assert.h>
 
 // Convenience function for checking CUDA runtime API results
 // can be wrapped around any runtime API call. No-op in release builds.
 inline cudaError_t checkCuda(cudaError_t result) {
 #if defined(DEBUG) || defined(_DEBUG)
   if (result != cudaSuccess) {
     fprintf(stderr, "CUDA Runtime Error: %s\n",
       cudaGetErrorString(result));
     assert(result == cudaSuccess);
   }
 #endif
   return result;
 }

bool FeatureGenerator::init(void* out_blob, 
    int height = 640, int width = 640, float range = 60.0, 
    float min_height = -5.0, float max_height = 5.0, 
    bool intensity_normalized = false, int batchsize = 1, 
    int channel = 8) {

  dev_out_blob_ = (float*)out_blob;
  width_   = width;
  height_  = height;
  channel_ = channel;  
  batch_   = batchsize;
  range_   = range;
  min_height_   = min_height;
  max_height_   = max_height;
  intensity_normalized_ = intensity_normalized; 
  channel_size_         = height_ * width_;
  channel_bytes_size_   = channel_size_ * sizeof(float);

  max_height_data_ = (float*)malloc(channel_bytes_size_);
  top_intensity_data_ = (float*)malloc(channel_bytes_size_);

  size_t log_table_bytes    = LOG_TABLE_SIZE * sizeof(float);
  // table for log(x+1) 
  checkCuda( cudaMalloc((void **)&dev_log_table_, log_table_bytes) );

  int channel_index = 0;

  dev_max_height_data_     = dev_out_blob_ + channel_size_ * channel_index++;
  dev_mean_height_data_    = dev_out_blob_ + channel_size_ * channel_index++;
  dev_count_data_          = dev_out_blob_ + channel_size_ * channel_index++;
  dev_direction_data_      = dev_out_blob_ + channel_size_ * channel_index++;
  dev_top_intensity_data_  = dev_out_blob_ + channel_size_ * channel_index++;
  dev_mean_intensity_data_ = dev_out_blob_ + channel_size_ * channel_index++;
  dev_distance_data_       = dev_out_blob_ + channel_size_ * channel_index++;
  dev_nonempty_data_       = dev_out_blob_ + channel_size_ * channel_index++;

  float* direction_data_; 
  float* distance_data_;
  float* log_table_;
  
  checkCuda( cudaMallocHost((void**)&log_table_, log_table_bytes) ); // host pinned
  checkCuda( cudaMallocHost((void**)&direction_data_, channel_bytes_size_) ); // host pinned
  checkCuda( cudaMallocHost((void**)&distance_data_, channel_bytes_size_) ); // host pinned

  for (int i = 0; i < LOG_TABLE_SIZE; ++i) {
    log_table_[i] = std::log1p((float)i);
  }

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
          static_cast<float>(std::hypot(center_x, center_y) / range_ - 0.5);
    }
  }

  checkCuda( cudaMemcpy(dev_log_table_, log_table_, 
      log_table_bytes, cudaMemcpyHostToDevice) );
  checkCuda( cudaMemcpy(dev_direction_data_, direction_data_, 
      channel_bytes_size_, cudaMemcpyHostToDevice) );
  checkCuda( cudaMemcpy(dev_distance_data_, distance_data_, 
      channel_bytes_size_, cudaMemcpyHostToDevice) );
  cudaFreeHost(log_table_);      
  cudaFreeHost(direction_data_);
  cudaFreeHost(distance_data_);

  return true;
}

// float FeatureGenerator::logCount(int count) {
//   if (count < static_cast<int>(log_table_.size())) {
//     return log_table_[count];
//   }
//   return std::log(static_cast<float>(1 + count));
// }

void FeatureGenerator::generate(const void* pointcloud_ptr, int num) {
    // const auto& points = pc_ptr->points;
    float inv_res_x = 0.5 * static_cast<float>(width_) / static_cast<float>(range_);
    float inv_res_y = 0.5 * static_cast<float>(height_) / static_cast<float>(range_);
    // dev_max_height_data_ don't need to be reset 
    checkCuda( cudaMemsetAsync((void*)dev_mean_height_data_, 0, channel_bytes_size_) ); 
    checkCuda( cudaMemsetAsync((void*)dev_count_data_, 0, channel_bytes_size_) ); 
    // dev_direction_data_ don't need to be reset 
    checkCuda( cudaMemsetAsync((void*)dev_top_intensity_data_, 0, channel_bytes_size_) ); 
    checkCuda( cudaMemsetAsync((void*)dev_mean_intensity_data_, 0, channel_bytes_size_) ); 
    // dev_distance_data_  don't need to be reset 
    checkCuda( cudaMemsetAsync((void*)dev_nonempty_data_, 0, channel_bytes_size_) ); 

    PointXYZI* dev_pc;
    unsigned int pc_size = num;
    size_t point_bytes_size = num * sizeof(PointXYZI);
    PointXYZI* pc_ptr = (PointXYZI*)pointcloud_ptr;

    // float* max_height_data_ = (float*)malloc(channel_bytes_size_);
    // float* top_intensity_data_ = (float*)malloc(channel_bytes_size_);
    
    for (int i=0; i<channel_size_; i++) {
      max_height_data_[i] = min_height_;
    } 
    memset((void*)top_intensity_data_, 0, channel_bytes_size_);

    for (int i = 0; i < num; ++i) {
      if (pc_ptr[i].z <= min_height_ || pc_ptr[i].z >= max_height_) {
        continue;
      }
      int pos_x = F2I(pc_ptr[i].y, range_, inv_res_x);  // col
      int pos_y = F2I(pc_ptr[i].x, range_, inv_res_y);  // row
      if (pos_x >= width_ || pos_x < 0 || pos_y >= height_ || pos_y < 0) {
        continue;
      }
      int idx = pos_y * width_ + pos_x;
      float pz = pc_ptr[i].z;
      float pi = (intensity_normalized_) ? pc_ptr[i].i : (pc_ptr[i].i / 255.0f);  
      if (max_height_data_[idx] < pz) {
        max_height_data_[idx] = pz;
        top_intensity_data_[idx] = pi;
      }
    } 

    cudaStream_t ss,sc1,sc2;
    cudaStreamCreate(&ss);
    cudaStreamCreate(&sc1);
    cudaStreamCreate(&sc2);

    checkCuda( cudaMalloc((void**)&dev_pc, point_bytes_size) );

    checkCuda( cudaMemcpyAsync((void*)dev_pc, (void*)pc_ptr, 
        point_bytes_size, cudaMemcpyHostToDevice, ss));

    cuda_mem_set<<<160, 160, 0, sc1>>>(dev_max_height_data_, 
        channel_size_, min_height_);

    cloud_other_cal<<<64, 128, 0 , ss>>>(dev_pc, pc_size, dev_mean_height_data_,
        dev_mean_intensity_data_, dev_count_data_, range_, width_, height_, 
        max_height_, min_height_, inv_res_x, inv_res_y);

    checkCuda( cudaMemcpyAsync(dev_max_height_data_, max_height_data_, 
        channel_bytes_size_, cudaMemcpyHostToDevice, sc1) );

    checkCuda( cudaMemcpyAsync(dev_top_intensity_data_, top_intensity_data_, 
        channel_bytes_size_, cudaMemcpyHostToDevice, sc2) );

    cudaStreamSynchronize(ss);
    cudaStreamSynchronize(sc1);
    cudaStreamSynchronize(sc2);
    
    cloud_average<<<160, 160>>>(dev_max_height_data_, dev_mean_height_data_, dev_count_data_,
        dev_mean_intensity_data_, dev_nonempty_data_, channel_size_,dev_log_table_);
    cudaStreamDestroy(ss);
    cudaStreamDestroy(sc1);
    cudaStreamDestroy(sc2);

    // free(max_height_data_);
    // free(top_intensity_data_);
    cudaFree(dev_pc);
}
 