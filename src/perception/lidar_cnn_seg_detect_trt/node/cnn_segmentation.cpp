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
#include "benchmark.h"

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

bool CNNSegmentation::init() {
  std::string engine_file;
  ros::NodeHandle private_node_handle("~");//to receive args
  benchmark_path_.clear();
  duration_path_.clear();

  //for SHM
  ROS_WARN("[CNNSeg] Enter SHM parameter read!");
  // private_node_handle.param<bool>("shm_enable", shm_enable_, false);
  
  if (private_node_handle.getParam("shm_enable", shm_enable_)){
    ROS_INFO_STREAM("shm enable is : " << shm_enable_);
  }
  if(shm_enable_){
      private_node_handle.param<int>("shm_key", shm_key_, 200);
      private_node_handle.param<int>("sem_proj_id", sem_proj_id_, 100);
      private_node_handle.param<int>("shm_clent_index", shm_clent_index_, 1);     //TODO
      private_node_handle.param<int>("shm_cycle_delay", shm_cycle_delay_, 70);     //TODO
      sem_id_ = sem_object_rec_.GetSemid(sem_proj_id_, 0);      //open sem
      if (sem_id_ == -1)
      {
        ROS_ERROR("semget open sem failed.");
      } 
      else{
        ROS_WARN_STREAM("[SEM] semget return value(SEM_ID) is " << sem_id_);
      }
      shm_obj_point_ = shared_memory::ShmObject{shm_key_};
      
      ROS_INFO_STREAM("shm key is : " << shm_key_ 
          << ", and sem project id is :" << sem_proj_id_
          << ", and shm clent index is :" << shm_clent_index_
          << ", and shm cycle delay is :" << shm_cycle_delay_);
  }


  if (private_node_handle.getParam("benchmark_path", benchmark_path_)){
    ROS_INFO("[%s] benchmark_path: %s", __APP_NAME__, benchmark_path_.c_str());
  }
  if (private_node_handle.getParam("duration_path", duration_path_)){
    ROS_INFO("[%s] duration_path: %s", __APP_NAME__, duration_path_.c_str());
  }
  if (private_node_handle.getParam("engine_file", engine_file)){
    ROS_INFO("[%s] duration_path: %s", __APP_NAME__, duration_path_.c_str());
  } else {
    ROS_ERROR("[%s] error path for engine_file.", __APP_NAME__);
    return false;
  }
  // private_node_handle.param<std::string>("benchmark_path", benchmark_path_, "");
  // private_node_handle.param<std::string>("duration_path", duration_path_, "");
  // private_node_handle.param<std::string>("engine_file", engine_file, "");  



  private_node_handle.param<std::string>("publish_objects", 
      topic_pub_objects_, "/detection/lidar_detector/objects");
  // private_node_handle.param<std::string>("publish_points", 
  //     topic_pub_points_, "/detection/lidar_detector/points_cluster");
  private_node_handle.param<std::string>("points_src", topic_src_, "/points_raw");
  private_node_handle.param<float>("range", range_, 50.);
  private_node_handle.param<float>("min_height", min_height_, -5.);
  private_node_handle.param<float>("max_height", max_height_, 5.);
  private_node_handle.param<bool>("intensity_normalized", intensity_normalized_, false);
  private_node_handle.param<float>("score_threshold", score_threshold_, 0.6);

  private_node_handle.param<float>("objectness_thresh", objectness_thresh_, 0.5);
  private_node_handle.param<float>("height_thresh", height_thresh_, 0.5);
  private_node_handle.param<int>("min_pts_num", min_pts_num_, 3);
  private_node_handle.param<bool>("use_all_grids_for_clustering", use_all_grids_for_clustering_, false);
  // private_node_handle.param<float>("ground_height", ground_height_, -1.8);
  
  // channel_ = 8;
  // batch_ = 1;
  // channel_size_ = height_ * width_;
  // objectness_thresh_ = 0.5;
  // height_thresh_ = 0.5;
  // min_pts_num_ = 3; 
  // use_all_grids_for_clustering_ = true;

  std::vector<std::string> outputBlobName{
      "category_score",
      "instance_pt", 
      "confidence_score", 
      "classify_score", 
      "heading_pt", 
      "height_pt"};
  output_blob_name_ = outputBlobName;
  std::vector<std::string> input_blob_name_{"input"};

  trt_net_.reset(novauto::tensorrt::inference::CreateInferenceByName(
    "TRTNet", engine_file, input_blob_name_, outputBlobName)); 

  std::vector<int> dims = (*trt_net_).GetBindingDims(0);
  width_   = dims[dims.size() - 1];
  height_  = dims[dims.size() - 2];
  channel_ = dims[dims.size() - 3];  
  batch_   = (*trt_net_).GetMaxBatchSize();
	channel_size_  = height_ * width_;

  feature_generator_.reset(new FeatureGenerator());

#ifdef __USE_GPU__
  ROS_INFO("[%s] preprocess use GPU.", __APP_NAME__);
  if ( !feature_generator_->init((*trt_net_).GetBindingPtr(0), height_,
       width_, range_, min_height_, max_height_, intensity_normalized_, 
       batch_, channel_) ) {
    ROS_ERROR("[%s] Fail to Initialize feature generator for CNNSegmentation", __APP_NAME__);
    return false;
  }
#else
  feature_blob_.reset(new std::vector<float>());
  if ( !feature_generator_->init(feature_blob_.get(), height_, width_, range_, 
      min_height_, max_height_, intensity_normalized_, batch_, channel_) ) {
    ROS_ERROR("[%s] Fail to Initialize feature generator for CNNSegmentation", __APP_NAME__);
    return false;
  }  
#endif

  cluster2d_.reset(new Cluster2D());
  if (!cluster2d_->init(height_, width_, range_)) {
    ROS_ERROR("[%s] Fail to Initialize cluster2d for CNNSegmentation", __APP_NAME__);
    return false;
  }
  output_data_.resize(output_blob_name_.size());

  ROS_INFO("[%s] engine_file: %s", __APP_NAME__, engine_file.c_str());
  ROS_INFO("[%s] publish_objects: %s", __APP_NAME__, topic_pub_objects_.c_str());
  // ROS_INFO("[%s] publish_points: %s", __APP_NAME__, topic_pub_points_.c_str());
  ROS_INFO("[%s] points_src: %s", __APP_NAME__, topic_src_.c_str());
  ROS_INFO("[%s] range: %.2f", __APP_NAME__, range_);
  ROS_INFO("[%s] min_height: %.2f", __APP_NAME__, min_height_);
  ROS_INFO("[%s] max_height: %.2f", __APP_NAME__, max_height_);
  ROS_INFO("[%s] intensity_normalized: %d", __APP_NAME__, int(intensity_normalized_));
  ROS_INFO("[%s] score_threshold: %.2f", __APP_NAME__, score_threshold_);
  // ROS_INFO("[%s] width: %d", __APP_NAME__, width_);
  // ROS_INFO("[%s] height: %d", __APP_NAME__, height_);
  // ROS_INFO("[%s] use_gpu: %d", __APP_NAME__, use_gpu_);
  // ROS_INFO("[%s] gpu_device_id: %d", __APP_NAME__, gpu_device_id_);
  // ROS_INFO("[%s] ground height: %.2f", __APP_NAME__, ground_height_);

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
#ifdef __USE_GPU__
  (*feature_generator_).generate((void*)pc_ptr->points.data(), 
      pc_ptr->points.size());
#else
	feature_generator_.get()->generate(pc_ptr);
  trt_net_->SetInputTensor(0, *feature_blob_);
#endif
  _TCSV_END();
  _TCSV_START();

  trt_net_->Infer();
#if 1
  for(int output_idx = 0; output_idx < output_blob_name_.size(); 
      ++output_idx) {
    trt_net_->GetOutputTensor(output_idx + 1, output_data_[output_idx]);  
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
  // float objectness_thresh = 0.5;
  // float height_thresh = 0.5;
  // int min_pts_num = 3; 
  // bool use_all_grids_for_clustering = true;

  cluster2d_->cluster(category_pt, instance_pt, pc_ptr,
      valid_idx, objectness_thresh_,  use_all_grids_for_clustering_);
  cluster2d_->filter(confidence_pt, height_pt);
  cluster2d_->classify(classify_pt);
  cluster2d_->heading(heading_pt);

  std::vector<nova::Object>  objects_loc;
  nova::Header  in_header;
  cluster2d_->getObjects(score_threshold_, height_thresh_,
      min_pts_num_, objects_loc, in_header);
	// std::cout << "objects size : " << objects_loc.size() << std::endl;

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

#endif
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
#ifdef __USE_GPU__
    (*feature_generator_).generate((void*)pc_ptr->points.data(), pc_ptr->size());
#else
    feature_generator_.get()->generate(pc_ptr);
    trt_net_->SetInputTensor(0, *feature_blob_);
#endif
    trt_net_->Infer();
    for(int output_idx = 0; 
        output_idx < output_blob_name_.size(); 
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

    // std::cout << "objects size : " << objects.size() << std::endl;
  }
}

// void CNNSegmentation::run() {
//   init();
//   points_sub_ = nh_.subscribe(topic_src_, 1, 
//       &CNNSegmentation::pointsCallback, this);
//   // points_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
//   //     topic_pub_points_, 2);
//   objects_pub_ = nh_.advertise<autoware_msgs::DetectedObjectArray>(
//       topic_pub_objects_, 2);
//   ROS_INFO("[%s] Ready. Waiting for data...", __APP_NAME__);
// }

void CNNSegmentation::run() {
  init();
  if(!shm_enable_){
      points_sub_ = nh_.subscribe(topic_src_, 1, 
          &CNNSegmentation::pointsCallback, this);
  }else{
      boost::shared_ptr<boost::thread> shm_thread = 
          boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&CNNSegmentation::pointsCallbackShm, this)));
  }
  // points_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
  //     topic_pub_points_, 2);
  objects_pub_ = nh_.advertise<autoware_msgs::DetectedObjectArray>(
      topic_pub_objects_, 2);
  ROS_INFO("[%s] Ready. Waiting for data...", __APP_NAME__);
}

void CNNSegmentation::pointsCallbackShm(){
  rslidar_pointcloud::rslidarPointStruct struct_of_point;
  bool sem_exit_flag = sem_object_rec_.IfSemExist(sem_proj_id_);
  while(sem_exit_flag){           
    sem_object_rec_.P(sem_id_, 0, -1);    
    read_data_ = shm_obj_point_.get_data_for_read_with_sem(shm_clent_index_);    //unsigned char*
    if(read_data_!=NULL)//表示已经获取到可读数据
    {
        std::memcpy(&struct_of_point, read_data_+MAX_CLIENT_NUM, rslidar_pointcloud::shm_data_length__);
        read_data_[shm_clent_index_-1] = 0;//读取完成标志位
        sensor_msgs::PointCloud2 point_msg = struct_of_point.get_point_msg();  

        //perception part start
        pcl::PointCloud<pcl::PointXYZI>::Ptr in_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(point_msg, *in_pc_ptr);
        pcl::PointIndices valid_idx;
        auto &indices = valid_idx.indices;
        indices.resize(in_pc_ptr->size());
        std::iota(indices.begin(), indices.end(), 0);
        message_header_ = point_msg.header;

        autoware_msgs::DetectedObjectArray objects;
        objects.header = message_header_;
        segment(in_pc_ptr, valid_idx, objects);

        // pubColoredPoints(objects);
        objects_pub_.publish(objects);

        if (!benchmark_path_.empty()) {
          // std::cout << "record" << std::endl;
          nova::Benchmark bench(nova::Benchmark::Dataset::Waymo, benchmark_path_);
          bench.record(objects);
        }
        //perception part end
        usleep(shm_cycle_delay_*1000);     
    }
    sem_object_rec_.V(sem_id_, 0, 1);
    usleep(2000);    
  }     
}   //end of pointsCallbackShm()

void CNNSegmentation::pointsCallback(const sensor_msgs::PointCloud2 &msg)
{
  ROS_INFO("[SHM] perception not in SHM");
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
    // std::cout << "record" << std::endl;
    nova::Benchmark bench(nova::Benchmark::Dataset::Waymo, benchmark_path_);
    bench.record(objects);
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
