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
#include "time_point.h"
#include "inference.h"
#include "inference_factory.h"
#include "cnn_segmentation.h"
#include <fstream>


static std::string GetTypeString(MetaType meta_type)  {
  switch (meta_type) {
    case META_UNKNOWN:
      return "Unknown";
    case META_SMALLMOT:
      return "Car";
    case META_BIGMOT:
      return "Truck";
    case META_NONMOT:
      return "Cyclist";
    case META_PEDESTRIAN:
      return "Pedestrian";
    default:
      return "unknown";
  }
}

CNNSegmentation::CNNSegmentation() : nh_()
{
}

bool CNNSegmentation::init()
{
  std::string proto_file_;
  std::string weight_file_;
  std::string deploy_mode_;
  ros::NodeHandle private_node_handle("~");//to receive args
  benchmark_path_.clear();
  if (private_node_handle.getParam("benchmark_path", benchmark_path_)){
    ROS_INFO("[%s] benchmark_path: %s", __APP_NAME__, benchmark_path_.c_str());
  }
  duration_path_.clear();
  if (private_node_handle.getParam("duration_path", duration_path_)){
    ROS_INFO("[%s] duration_path: %s", __APP_NAME__, duration_path_.c_str());
  }
  if (private_node_handle.getParam("network_definition_file", proto_file_)){
    ROS_INFO("[%s] network_definition_file: %s", __APP_NAME__, proto_file_.c_str());
  } else {
    ROS_INFO("[%s] No Network Definition File was received. Finishing execution.", __APP_NAME__);
    return false;
  }
  if (private_node_handle.getParam("pretrained_model_file", weight_file_)){
    ROS_INFO("[%s] Pretrained Model File: %s", __APP_NAME__, weight_file_.c_str());
  } else{
    ROS_INFO("[%s] No Pretrained Model File was received. Finishing execution.", __APP_NAME__);
    return false;
  }

  private_node_handle.param<std::string>("points_src", topic_src_, "points_raw");
  ROS_INFO("[%s] points_src: %s", __APP_NAME__, topic_src_.c_str());

  private_node_handle.param<double>("range", range_, 60.);
  ROS_INFO("[%s] range: %.2f", __APP_NAME__, range_);

  private_node_handle.param<double>("score_threshold", score_threshold_, 0.6);
  ROS_INFO("[%s] score_threshold: %.2f", __APP_NAME__, score_threshold_);

  private_node_handle.param<int>("width", width_, 640);
  ROS_INFO("[%s] width: %d", __APP_NAME__, width_);

  private_node_handle.param<int>("height", height_, 640);
  ROS_INFO("[%s] height: %d", __APP_NAME__, height_);

  private_node_handle.param<bool>("use_gpu", use_gpu_, false);
  ROS_INFO("[%s] use_gpu: %d", __APP_NAME__, use_gpu_);

  private_node_handle.param<int>("gpu_device_id", gpu_device_id_, 0);
  ROS_INFO("[%s] gpu_device_id: %d", __APP_NAME__, gpu_device_id_);

  private_node_handle.param<double>("ground_height", ground_height_, -1.73);
  ROS_INFO("[%s] gpu_device_id: %d", __APP_NAME__, ground_height_);

  private_node_handle.param<std::string>("deploy_mode", deploy_mode_, "FP32");
  ROS_INFO("[%s] deploy_mode: %d", __APP_NAME__, deploy_mode_.c_str());

  batch_ = 1;
  channel_size_ = height_ * width_;

  cluster2d_.reset(new Cluster2D());
  if (!cluster2d_->init(height_, width_, range_)) {
    ROS_ERROR("[%s] Fail to Initialize cluster2d for CNNSegmentation", __APP_NAME__);
    return false;
  }

  feature_generator_.reset(new FeatureGenerator());
  feature_blob_.reset(new std::vector<float>());
  if (!feature_generator_->init(feature_blob_.get())) {
    ROS_ERROR("[%s] Fail to Initialize feature generator for CNNSegmentation", __APP_NAME__);
    return false;
  }

  int mode = 0;
  std::string engine_file;
  if (deploy_mode_ == "FP16") {
    engine_file = weight_file_+"_FP16.trt";
    mode = 1;
  } else if (deploy_mode_ == "INT8") {
    engine_file = weight_file_+"_INT8.trt";
    mode = 2;
  } else { // deploy_mode_ == "FP32"
    engine_file = weight_file_+"_FP32.trt";
    mode = 0;
  }

  std::vector<std::string> outputBlobName{
      "category_score",
      "instance_pt", 
      "confidence_score", 
      "classify_score", 
      "heading_pt", 
      "height_pt"};
  output_blob_name_.clear();
  output_blob_name_ = outputBlobName;
  std::vector<std::string> input_blob_name_{"input"};
  trt_net_.reset(novauto::tensorrt::inference::CreateInferenceByName(
    "TRTNet", engine_file, input_blob_name_, outputBlobName)); 
  

  // caffe_net_.reset(new Trt());
  // caffe_net_->CreateEngine(proto_file_, weight_file_, 
  //     engine_file, outputBlobName, batch_, mode);
  output_data_.resize(output_blob_name_.size());

  return true;
}


bool CNNSegmentation::segment(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_ptr,
    const pcl::PointIndices &valid_idx, autoware_msgs::DetectedObjectArray &objects) {
  int num_pts = static_cast<int>(pc_ptr->points.size());
  if (num_pts == 0) {
    ROS_INFO("[%s] Empty point cloud.", __APP_NAME__);
    return true;
  }
  _TCSV_INIT();
  _TCSV_START();
	feature_generator_.get()->generate(pc_ptr);
  trt_net_->SetInputTensor(0, *feature_blob_);
  _TCSV_END();
  _TCSV_START();
  trt_net_->Infer();
  for(int output_idx = 0; 
      output_idx < output_blob_name_.size(); 
      ++output_idx) {
    trt_net_->GetOutputTensor(output_idx + 1, output_data_[output_idx]);
    // caffe_net_->DataTransfer(output_data_[output_idx - 1], output_idx, false);    
  }
  _TCSV_END();
  _TCSV_START();
  float *category_pt = output_data_[0].data();
  float *instance_pt = output_data_[1].data();
  float *confidence_pt = output_data_[2].data();
  float *classify_pt = output_data_[3].data();
  float *heading_pt = output_data_[4].data();
  float *height_pt = output_data_[5].data();

  // clutser points and construct segments/objects
  float objectness_thresh = 0.5;
  bool use_all_grids_for_clustering = true;
  cluster2d_->cluster(category_pt,
      instance_pt,
      pc_ptr,
      valid_idx,
      objectness_thresh, 
      use_all_grids_for_clustering);
  cluster2d_->filter(confidence_pt, height_pt);
  cluster2d_->classify(classify_pt);
  cluster2d_->heading(heading_pt);
  float height_thresh = 0.5;
  int min_pts_num = 3;
  std::vector<nova::Object>  objects_loc;
  nova::Header  in_header;
  cluster2d_->getObjects(score_threshold_,
      height_thresh,
      min_pts_num,
      objects_loc,
      in_header);

	std::cout << "objects size : " << objects_loc.size() << std::endl;
  _TCSV_END();
  _TCSV_PRINT(duration_path_, (!duration_path_.empty()) );
  for (auto& obj: objects_loc) {
    autoware_msgs::DetectedObject resulting_object;
    resulting_object.header = message_header_;
    resulting_object.score = obj.score;
    resulting_object.label = GetTypeString((MetaType)obj.label);
    resulting_object.angle = obj.yaw;
    resulting_object.valid = true;
    resulting_object.pose_reliable = true;
    resulting_object.color.r = 0;
    resulting_object.color.g = 255;
    resulting_object.color.b = 0;
    resulting_object.color.a = 0.2;
    resulting_object.pose.position.x = obj.pose.x();
    resulting_object.pose.position.y = obj.pose.y();
    resulting_object.pose.position.z = obj.pose.z();
    resulting_object.dimensions.x = obj.dims.x();
    resulting_object.dimensions.y = obj.dims.y();
    resulting_object.dimensions.z = obj.dims.z();
    resulting_object.pose.orientation.x = obj.orie.x();
    resulting_object.pose.orientation.y = obj.orie.y();
    resulting_object.pose.orientation.z = obj.orie.z();
    resulting_object.pose.orientation.w = obj.orie.w();
    resulting_object.space_frame = message_header_.frame_id;
    objects.objects.push_back(resulting_object);
  }
  
  return true;
}

void CNNSegmentation::test_run() {
  init();
  std::string data_path = "/home/nova/shared_dir/examples/cnn_seg_trt/data/1573712277.789873000.pcd";
  pcl::PointCloud<pcl::PointXYZI>::Ptr pc_ptr (new pcl::PointCloud<pcl::PointXYZI>); 
	if (pcl::io::loadPCDFile<pcl::PointXYZI> (data_path.c_str(), *pc_ptr) == -1) {
		PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
	}

  for (int k=0; k<100; k++) {  
    
    feature_generator_.get()->generate(pc_ptr);
    trt_net_->Infer();
    for(int output_idx = 1; 
        output_idx <= output_blob_name_.size(); 
        ++output_idx) {
      trt_net_->GetOutputTensor(output_idx + 1, output_data_[output_idx]);   
    }
    float *category_pt = output_data_[0].data();
    float *instance_pt = output_data_[1].data();
    float *confidence_pt = output_data_[2].data();
    float *classify_pt = output_data_[3].data();
    float *heading_pt = output_data_[4].data();
    float *height_pt = output_data_[5].data();
    std::shared_ptr<Cluster2D> cluster;
    cluster.reset(new Cluster2D);
    std::vector<nova::Object>  objects;

    pcl::PointIndices valid_idx;
    auto &indices = valid_idx.indices;
    indices.resize(pc_ptr->size());
    std::iota(indices.begin(), indices.end(), 0);
    nova::Header in_header;
    cluster->init(height_, width_, range_);
    cluster->cluster(category_pt,
        instance_pt, pc_ptr, valid_idx, 0.5, true);
    cluster->filter(confidence_pt, height_pt);
    cluster->classify(classify_pt);
    cluster->heading(heading_pt);
    int min_pts_num = 3;
    cluster->getObjects(0.6,
        0.5, min_pts_num, objects, in_header);

    std::cout << "objects size : " << objects.size() << std::endl;
  }
}

void CNNSegmentation::run() {
  init();
  points_sub_ = nh_.subscribe(topic_src_, 1, &CNNSegmentation::pointsCallback, this);
  points_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/detection/lidar_detector/points_cluster", 1);
  objects_pub_ = nh_.advertise<autoware_msgs::DetectedObjectArray>("/detection/lidar_detector/objects", 1);
  ROS_INFO("[%s] Ready. Waiting for data...", __APP_NAME__);
}

void CNNSegmentation::pointsCallback(const sensor_msgs::PointCloud2 &msg)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr in_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(msg, *in_pc_ptr);
  pcl::PointIndices valid_idx;
  auto &indices = valid_idx.indices;
  indices.resize(in_pc_ptr->size());
  std::iota(indices.begin(), indices.end(), 0);
  message_header_ = msg.header;

  autoware_msgs::DetectedObjectArray objects;
  objects.header = message_header_;
  segment(in_pc_ptr, valid_idx, objects);

  // pubColoredPoints(objects);
  objects_pub_.publish(objects);

  if (!benchmark_path_.empty()) {
      std::stringstream s_file;
      s_file << std::setw(4) 
          << std::setfill('0') 
          << objects.header.seq ;
      std::string file = benchmark_path_ + s_file.str() + ".txt";
      std::remove(file.c_str());
      std::ofstream outputfile(file, 
          std::ofstream::out | std::ofstream::app);
      for (auto& obj : objects.objects) {
        if (obj.pose.position.x > 50.0 || obj.pose.position.x < 0.0 
            || obj.pose.position.y > 25.0 || obj.pose.position.y < -25.0
            || obj.dimensions.x > 10.0 || obj.dimensions.y > 4.0) {
          continue;
        }
        outputfile << obj.label << " " 
            << obj.score << " " 
            << obj.pose.position.x << " "
            << obj.pose.position.y << " "
            << obj.pose.position.z << " "
            << obj.dimensions.x << " "
            << obj.dimensions.y << " "
            << obj.dimensions.z << " "
            // << 0.0 << std::endl;
            << -obj.angle << std::endl;
      }
  }
}

void CNNSegmentation::pubColoredPoints(
    const autoware_msgs::DetectedObjectArray &objects_array) {
  pcl::PointCloud<pcl::PointXYZRGB> colored_cloud;
  for (size_t object_i = 0; 
      object_i < objects_array.objects.size(); 
      object_i++) {
    // std::cout << "objct i" << object_i << std::endl;
    pcl::PointCloud<pcl::PointXYZI> object_cloud;
    pcl::fromROSMsg(objects_array.objects[object_i].pointcloud, object_cloud);
    int red = (object_i) % 256;
    int green = (object_i * 7) % 256;
    int blue = (object_i * 13) % 256;

    for (size_t i = 0; i < object_cloud.size(); i++) {
      pcl::PointXYZRGB colored_point;
      colored_point.x = object_cloud[i].x;
      colored_point.y = object_cloud[i].y;
      colored_point.z = object_cloud[i].z;
      colored_point.r = red;
      colored_point.g = green;
      colored_point.b = blue;
      colored_cloud.push_back(colored_point);
    }
  }
  sensor_msgs::PointCloud2 output_colored_cloud;
  pcl::toROSMsg(colored_cloud, output_colored_cloud);
  output_colored_cloud.header = message_header_;
  points_pub_.publish(output_colored_cloud);
}
