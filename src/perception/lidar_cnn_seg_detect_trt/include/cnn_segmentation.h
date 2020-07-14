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
#ifndef CNN_SEGMENTATION_H
#define CNN_SEGMENTATION_H

#define __USE_GPU__ 1

#ifdef __USE_GPU__
  #include "feature_generator_cuda.h"
#else
  #include "feature_generator.h"
#endif
#include "cluster2d.h"

#include "inference.h"
#include "inference_factory.h"
#include <chrono>
#include <numeric>

#include <ros/ros.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PointIndices.h>
#include <pcl_ros/point_cloud.h>

#include <autoware_msgs/DetectedObjectArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>

#include <std_msgs/Header.h>



#define __APP_NAME__ "lidar_apollo_cnn_seg_detect"

class CNNSegmentation
{
public:
  CNNSegmentation();
  ~CNNSegmentation() {}

  void run();
  void test_run();

private:
  std::string benchmark_path_;
  std::string duration_path_;

  int width_;
  int height_;
  int channel_;
  int batch_;
  int channel_size_;

  float score_threshold_;
  float ground_height_;  
  float range_;
  bool intensity_normalized_;
  float min_height_;
  float max_height_;
  float objectness_thresh_;
  float height_thresh_;
  int min_pts_num_; 
  bool use_all_grids_for_clustering_;

  std_msgs::Header message_header_;
  std::string topic_src_;
  std::string topic_pub_objects_;
  std::string topic_pub_points_;

  int gpu_device_id_;
  bool use_gpu_;

  // nodehandle
  ros::NodeHandle nh_;

  // publisher
  ros::Publisher points_pub_;;
  ros::Publisher objects_pub_;

  // subscriber
  ros::Subscriber points_sub_;

  std::shared_ptr<novauto::tensorrt::inference::Inference> trt_net_;
  std::vector<std::string> output_blob_name_;
  std::vector<std::vector<float>> output_data_;
  std::shared_ptr<std::vector<float>> feature_blob_;
  // clustering model for post-processing
  std::shared_ptr<Cluster2D> cluster2d_;
  // bird-view raw feature generator
  std::shared_ptr<FeatureGenerator> feature_generator_;

  bool init();

  bool segment(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_ptr,
               const pcl::PointIndices &valid_idx,
               autoware_msgs::DetectedObjectArray &objects);

  void pointsCallback(const sensor_msgs::PointCloud2 &msg);

  void pubColoredPoints(const autoware_msgs::DetectedObjectArray &objects);

};

#endif //CNN_SEGMENTATION_H
