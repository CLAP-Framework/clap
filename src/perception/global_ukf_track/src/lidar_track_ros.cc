// #include "lidar_track.h"
// #include "lidar_track.h"


// #include "lidar_track.h"
#include "lidar_track_ros.h"
#include "autoware_msgs/DetectedObject.h"
#include "autoware_msgs/DetectedObjectArray.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <Eigen/Dense>

#include <vector>
#include <chrono>
#include <string.h>
#include <stdio.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_listener.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <yaml-cpp/yaml.h>

#include <zzz_perception_msgs/TrackingBox.h>
#include <zzz_perception_msgs/TrackingBoxArray.h>

#define GLOBAL_TRACK 1
#define DEBUG_OUTPUT 0
#define USE_MESSAGE_FILTERS 0

// 'Car', 'Van', 'Truck','Pedestrian', 'Person_sitting', 'Cyclist','Tram',  'Misc' or  'DontCare'
static std::string Label2Classification[5] = {
    "Unknown",
    "Car",  
    "Pedestrian", 
    "Cyclist",
    "Truck"
};
int LidarClassification2Label(std::string c) {
    if (c == "Car") return 1;
    if (c == "Truck") return 4;
    if (c == "Pedestrian") return 2;
    if (c == "Cyclist") return 3;
    // else if (c == "nothing") return 0;
    return 0;
}

LidarTrackRos::LidarTrackRos() : private_nh_("~"){
    /** tracker id */
    track_id_ = 1;
    /** time step */
    dt_ = 0.0;
    /** initialized flag, default false */
    initialized_ = false;
    /** match distance threshold sqrt */
    dis_sqrt_thres_ = 16.0;
    /** birth threshold */
    birth_thres_ = 3;
    /** die threshold */
    die_thres_ = 3;
    /** default local output */
    private_nh_.param<bool>("global_output", global_output_,
            false);
    private_nh_.param<std::string>("input_obj_topic", input_obj_topic_,
            "/detection/lidar_detector/objects");
    if (global_output_) {
      std::string rotation_str;
      std::string translation_str;
      private_nh_.param<std::string>("input_odm_topic", input_odm_topic_,
          "/Odometry/data");
      private_nh_.param<std::string>("lidar2imu_rotation", rotation_str,
          "[-0.027756, -0.99923, -0.027761, 0.9991, -0.026836, -0.032968, 0.032197, -0.028651, 0.99907]");  
      private_nh_.param<std::string>("lidar2imu_translation", translation_str,
          "[-0.1798, 0.91685, 1.585]");  

      YAML::Node rot = YAML::Load(rotation_str);
      YAML::Node trans = YAML::Load(translation_str);
      lidar2imu_rotation_ << rot[0].as<double>(), rot[1].as<double>(), rot[2].as<double>(), 
          rot[3].as<double>(), rot[4].as<double>(), rot[5].as<double>(), 
          rot[6].as<double>(), rot[7].as<double>(), rot[8].as<double>();
      lidar2imu_translation_ << trans[0].as<double>(), trans[1].as<double>(), trans[2].as<double>();
    }
    private_nh_.param<std::string>("output_topic", output_topic_, 
            "/detection/lidar_tracker/objects");

    private_nh_.param<std::string>("output_zzz_topic", output_zzz_topic_,
            "/zzz/perception/objects_tracked");

    private_nh_.param<double>("dis_sqrt_thres", dis_sqrt_thres_, 16.0);
    private_nh_.param<int>("birth_thres", birth_thres_, 3);
    private_nh_.param<int>("die_thres", die_thres_, 3);
 
    private_nh_.param<bool>("is_benchmark", is_benchmark_, false);
    private_nh_.param<std::string>("kitti_data_dir", kitti_data_dir_, "");

    ROS_INFO("[%s] input_topic: %s", __APP_NAME__, input_obj_topic_.c_str());
    if (global_output_) {
        ROS_INFO("[%s] input_odm_topic: %s", __APP_NAME__, input_odm_topic_.c_str());
        ROS_INFO("[%s] lidar2imu_rotation:", __APP_NAME__);
        std::cout << lidar2imu_rotation_ << std::endl;
        ROS_INFO("[%s] lidar2imu_translation:", __APP_NAME__);
        std::cout << lidar2imu_translation_ << std::endl;
    }
    ROS_INFO("[%s] output_topic: %s", __APP_NAME__, output_topic_.c_str());
    ROS_INFO("[%s] dis_sqrt_thres: %s", __APP_NAME__, std::to_string(dis_sqrt_thres_).c_str());
    ROS_INFO("[%s] birth_thres: %s", __APP_NAME__, std::to_string(birth_thres_).c_str());
    ROS_INFO("[%s] die_thres: %s", __APP_NAME__, std::to_string(die_thres_).c_str());

    if (is_benchmark_) {
        std::remove(kitti_data_dir_.c_str());
    }
}

LidarTrackRos::~LidarTrackRos() {}

void LidarTrackRos::Run()
{
  pub_object_array_ = node_handle_.advertise<autoware_msgs::DetectedObjectArray>(
        output_topic_, 1);

  pub_zzz_object_array_ = node_handle_.advertise<zzz_perception_msgs::TrackingBoxArray>(
        output_zzz_topic_, 1);

  if (global_output_) {
#if USE_MESSAGE_FILTERS
    mf_sub_detected_array_ = new message_filters::Subscriber<nav_msgs::Odometry>(
        node_handle_, input_odm_topic_, 2);
    mf_sub_odm_ = new message_filters::Subscriber<autoware_msgs::DetectedObjectArray>(
        node_handle_, input_obj_topic_, 2);
    sync_ = new message_filters::Synchronizer<ObjectsSyncPolicy>(ObjectsSyncPolicy(10),
        *mf_sub_detected_array_, *mf_sub_odm_);
    sync_->registerCallback(boost::bind(&LidarTrackRos::syncCallback, 
        this, _1, _2));
#else // !USE_MESSAGE_FILTERS
    sub_detected_array_ = node_handle_.subscribe(input_obj_topic_, 
        1, &LidarTrackRos::objectsCallback, this);
    sub_odm_ = node_handle_.subscribe(input_odm_topic_, 
        1, &LidarTrackRos::odmCallback, this);
#endif // USE_MESSAGE_FILTERS
  } else {
    sub_detected_array_ = node_handle_.subscribe(input_obj_topic_, 
        1, &LidarTrackRos::objectsCallback, this);
  }
}

void LidarTrackRos::odmCallback(const nav_msgs::Odometry& input) {
  input_odm_ = input;
#if 1
  input_odm_.header.frame_id = "map";
#endif
}

void convert_DetectedObject2TrackingBox(
  const autoware_msgs::DetectedObject input,
  zzz_perception_msgs::TrackingBox &obs_box )
{
  zzz_perception_msgs::ObjectClass t;

  if (!strcasecmp("car", input.label.c_str())) {
    t.classid = 1;
  } else if (!strcasecmp("truck", input.label.c_str())) {
    t.classid = 49;
  } else if (!strcasecmp("Pedestrian", input.label.c_str())) {
    t.classid = 2;
  } else if (!strcasecmp("cyclist", input.label.c_str())) {
    t.classid = 3;
  } else {
    // unknown id
    t.classid = 0;
  }

  //std::cout << "$$$$$$$ " << t.classid << ", " << input.label << std::endl;

  obs_box.classes.push_back(t);
  obs_box.classes[0].score = input.score;
  obs_box.uid = input.id;
  obs_box.confidence = 1.0;

  // pose
  // (THU-meiyuan map origin (442867, 4427888)
  // shougang map origin  (428191, 4417667)
  obs_box.bbox.pose.pose.position.x = input.pose.position.x; // - 428191;
  obs_box.bbox.pose.pose.position.y = input.pose.position.y; // - 4417667;
  obs_box.bbox.pose.pose.position.z = input.pose.position.z;
  // orientation
  obs_box.bbox.pose.pose.orientation.x = input.pose.orientation.x;
  obs_box.bbox.pose.pose.orientation.y = input.pose.orientation.y;
  obs_box.bbox.pose.pose.orientation.z = input.pose.orientation.z;
  obs_box.bbox.pose.pose.orientation.w = input.pose.orientation.w;

  // TODO default value should be changed.
  obs_box.bbox.dimension.length_x = input.dimensions.x;
  obs_box.bbox.dimension.length_y = input.dimensions.y;
  obs_box.bbox.dimension.length_z = input.dimensions.z;
  // twist
  obs_box.twist.twist.linear.x = input.velocity.linear.x;
  obs_box.twist.twist.linear.y = input.velocity.linear.y;
  obs_box.twist.twist.linear.z = input.velocity.linear.z;
}

void convert_DetectedObjectArray2TrackingBoxArray(
  const autoware_msgs::DetectedObjectArray detected_objects_output, 
  zzz_perception_msgs::TrackingBoxArray &converted_objects_output )
{
  for (size_t i = 0; i < detected_objects_output.objects.size(); i++)
  {
    zzz_perception_msgs::TrackingBox output;
    convert_DetectedObject2TrackingBox(detected_objects_output.objects[i], output);
    converted_objects_output.targets.push_back(output);
  }
}

void LidarTrackRos::objectsCallback(const autoware_msgs::DetectedObjectArray& input) {
  input_header_ = input.header;
  if ( global_output_ && std::abs(input_header_.stamp.toSec() -
      input_odm_.header.stamp.toSec()) > 0.5) { 
    ROS_ERROR("Odometry lost !!!");
    return ;
  } 
  timestamp_ = input.header.stamp.sec * 1000000 + input.header.stamp.nsec / 1000;
  if (input.objects.size() > 0) {
      input_object_ = input.objects.at(0);
  }
  autoware_msgs::DetectedObjectArray detected_objects_output;
  SetDetectedObjects(input);
  Track();
  GetTrackObjects(detected_objects_output);

  zzz_perception_msgs::TrackingBoxArray trackingBoxArray;
  trackingBoxArray.header = input.header;
  trackingBoxArray.header.frame_id = "map";
  convert_DetectedObjectArray2TrackingBoxArray(detected_objects_output, trackingBoxArray);

  pub_object_array_.publish(detected_objects_output);
  // publish cnn-seg objects.
  pub_zzz_object_array_.publish(trackingBoxArray);

  // std::cout << "obj " << detected_objects_output.objects.size() << std::endl;
  if (is_benchmark_) {
    DumpResultText(detected_objects_output);
  }
}

void LidarTrackRos::syncCallback(const nav_msgs::Odometry::ConstPtr& pOdom, 
    const autoware_msgs::DetectedObjectArray::ConstPtr& pObj) {
  odmCallback(*pOdom);
  objectsCallback(*pObj);
}

void LidarTrackRos::SetDetectedObjects(const autoware_msgs::DetectedObjectArray& input) {
  in_objs_.clear();

  for (size_t i=0; i<input.objects.size(); i++) {
    autoware_msgs::DetectedObject obj = input.objects.at(i);
    // std::cout << "++++++++++++ " << obj.label << std::endl;
    bool has_same_obj = false;
    for (size_t j=0; j<i; j++) {
      if( fabs(obj.pose.position.x - input.objects.at(j).pose.position.x) < 0.1 
          && fabs(obj.pose.position.y - input.objects.at(j).pose.position.y) < 0.1 ) {
        has_same_obj = true;
        break;
      }
    }
    if (has_same_obj) {  
      continue;
    }
    ele::LidarTrackObject tmp;
#if GLOBAL_TRACK
    if (global_output_) {
      input_header_.frame_id = input_odm_.header.frame_id;
      Eigen::Quaterniond q_obj(
          obj.pose.orientation.w,
          obj.pose.orientation.x,
          obj.pose.orientation.y,
          obj.pose.orientation.z);
      Eigen::Quaterniond q_ego(
          input_odm_.pose.pose.orientation.w,
          input_odm_.pose.pose.orientation.x,
          input_odm_.pose.pose.orientation.y,
          input_odm_.pose.pose.orientation.z);
#if 1
      Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()));
      Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()));
      Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitZ())); 
      Eigen::Quaterniond rot_q;
      rot_q = yawAngle*pitchAngle*rollAngle;
      q_ego = rot_q * q_ego;
#endif
      Eigen::Vector3d pose_ego(
          input_odm_.pose.pose.position.x,
          input_odm_.pose.pose.position.y,
          input_odm_.pose.pose.position.z);
      Eigen::Vector3d pose_obj(
          obj.pose.position.x,
          obj.pose.position.y,
          obj.pose.position.z);
      Eigen::Quaterniond q_lidar2imu(lidar2imu_rotation_);

      Eigen::Vector3d pose_obj_global = pose_ego + q_ego.toRotationMatrix() * 
          (lidar2imu_translation_ + lidar2imu_rotation_ * pose_obj);
      Eigen::Quaterniond q_pbj_global = q_ego * q_lidar2imu * q_obj;
      tmp.pose.x = pose_obj_global.x();
      tmp.pose.y = pose_obj_global.y();
      tmp.pose.z = pose_obj_global.z();
      tmp.orientation.x = q_pbj_global.x();
      tmp.orientation.y = q_pbj_global.y();
      tmp.orientation.z = q_pbj_global.z();
      tmp.orientation.w = q_pbj_global.w();  
#if DEBUG_OUTPUT
      std::cout << "obj -> global " << i << " : " << obj.pose.position.x << " " << obj.pose.position.y
          << " -> " << tmp.pose.x << " " << tmp.pose.y 
          << std::endl;
#endif 
    } else {
      tmp.pose.x = obj.pose.position.x;
      tmp.pose.y = obj.pose.position.y;
      tmp.pose.z = obj.pose.position.z;
      tmp.orientation.x = obj.pose.orientation.x;
      tmp.orientation.y = obj.pose.orientation.y;
      tmp.orientation.z = obj.pose.orientation.z;
      tmp.orientation.w = obj.pose.orientation.w;      
    }
#else // GLOBAL_TRACK
    tmp.pose.x = obj.pose.position.x;
    tmp.pose.y = obj.pose.position.y;
    tmp.pose.z = obj.pose.position.z;
    tmp.orientation.x = obj.pose.orientation.x;
    tmp.orientation.y = obj.pose.orientation.y;
    tmp.orientation.z = obj.pose.orientation.z;
    tmp.orientation.w = obj.pose.orientation.w; 
#endif // GLOBAL_TRACK
    tmp.dimension.x = obj.dimensions.x;
    tmp.dimension.y = obj.dimensions.y;
    tmp.dimension.z = obj.dimensions.z;    
    tmp.label = LidarClassification2Label(obj.label);
    tmp.score = obj.score;
    tmp.timestamp = timestamp_;
    tmp.color.a = obj.color.a;
    tmp.color.r = obj.color.r;
    tmp.color.g = obj.color.g;
    tmp.color.b = obj.color.b;
    tmp.features.c.x = tmp.pose.x;
    tmp.features.c.y = tmp.pose.y;
    // tmp.features.fl.x = tmp.pose.x + obj.dimensions.x*0.5;
    // tmp.features.fl.y = tmp.pose.y + obj.dimensions.y*0.5;
    // tmp.features.rr.x = tmp.pose.x - obj.dimensions.x*0.5;
    // tmp.features.rr.y = tmp.pose.y - obj.dimensions.y*0.5;
    Eigen::Quaterniond quaternion(tmp.orientation.w, 
                                  tmp.orientation.x,
                                  tmp.orientation.y,
                                  tmp.orientation.z);
    // Z-Y-X RPY
    Eigen::Vector3d eulerAngle = quaternion.matrix().eulerAngles(2,1,0);
    tmp.features.yaw = eulerAngle(0);
    in_objs_.push_back(tmp);
  }
}

void LidarTrackRos::GetTrackObjects(
    autoware_msgs::DetectedObjectArray& output) {
  output.header = input_header_;  

  // for (auto & tmp : out_objs_) {
  for (size_t i=0; i<out_objs_.size(); i++) {
    ele::LidarTrackObject& tmp = out_objs_.at(i);
    /** remove same object */
    bool has_same_obj = false;
    for (size_t j=0; j<i; j++) {
      if( fabs(tmp.pose.x - out_objs_.at(j).pose.x) < 0.2 
          && fabs(tmp.pose.y - out_objs_.at(j).pose.y) < 0.2 ) {
        has_same_obj = true;
        break;
      }
    }
    if (has_same_obj) {  
      continue;
    }

    autoware_msgs::DetectedObject obj;
#if GLOBAL_TRACK
    if (global_output_) {
      output.header.frame_id = input_odm_.header.frame_id;
    }
    obj.pose.position.x = tmp.pose.x;
    obj.pose.position.y = tmp.pose.y;
    obj.pose.position.z = tmp.pose.z;
    obj.velocity.linear.x = tmp.velocity.x;
    obj.velocity.linear.y = tmp.velocity.y;
    obj.velocity.linear.z = 0.0;
    obj.pose.orientation.w = tmp.orientation.w;      
    obj.pose.orientation.x = tmp.orientation.x;
    obj.pose.orientation.y = tmp.orientation.y;
    obj.pose.orientation.z = tmp.orientation.z;
#if DEBUG_OUTPUT
    std::cout << "global " << i << " : " << obj.pose.position.x << " " << obj.pose.position.y
        << " v: " << obj.velocity.linear.x << " " << obj.velocity.linear.y 
        << std::endl;
#endif
#else // GLOBAL_TRACK
    if (global_output_) {
      output.header.frame_id = input_odm_.header.frame_id;
      /** ego vehicle Quaterniond and Position */
      Eigen::Quaterniond q_ego(input_odm_.pose.pose.orientation.w,
          input_odm_.pose.pose.orientation.x,
          input_odm_.pose.pose.orientation.y,
          input_odm_.pose.pose.orientation.z); 
      Eigen::Vector3d pose_ego(input_odm_.pose.pose.position.x,
          input_odm_.pose.pose.position.y, input_odm_.pose.pose.position.z);  

      /** ego vehicle Quaterniond, Position and Velocity */            
      Eigen::Quaterniond q_obj(tmp.orientation.w,
          tmp.orientation.x, tmp.orientation.y, tmp.orientation.z);
      Eigen::Vector3d pose_obj(tmp.pose.x, tmp.pose.y, tmp.pose.z);
      Eigen::Vector3d ref_vel_obj(tmp.velocity.x, tmp.velocity.y, tmp.velocity.z);
      Eigen::Quaterniond q_lidar2imu(lidar2imu_rotation_);
      Eigen::Quaterniond q_pbj_global = q_ego * q_lidar2imu * q_obj;
      Eigen::Vector3d pose_obj_global = pose_ego + q_ego.toRotationMatrix() * 
          (lidar2imu_translation_ + lidar2imu_rotation_ * pose_obj);

      Eigen::Vector3d vel_ego_global(input_odm_.twist.twist.linear.x,
          input_odm_.twist.twist.linear.y, input_odm_.twist.twist.linear.z);
      Eigen::Vector3d vel_obj_global = q_ego.toRotationMatrix() * 
          lidar2imu_rotation_ * ref_vel_obj + vel_ego_global;
#if DEBUG_OUTPUT
      std::cout << "objV -> globalV " << i << " : " << tmp.velocity.x << " " << tmp.velocity.y
          << " -> " << vel_obj_global.x() << " " << vel_obj_global.y() 
          << std::endl;
#endif 
      obj.pose.position.x = pose_obj_global.x();
      obj.pose.position.y = pose_obj_global.y();
      obj.pose.position.z = pose_obj_global.z();
      obj.velocity.linear.x = vel_obj_global.x();
      obj.velocity.linear.y = vel_obj_global.y();
      obj.velocity.linear.z = 0.0;
      obj.pose.orientation.w = q_pbj_global.w();
      obj.pose.orientation.x = q_pbj_global.x();
      obj.pose.orientation.y = q_pbj_global.y();
      obj.pose.orientation.z = q_pbj_global.z();
    } else {
      obj.pose.position.x = tmp.pose.x;
      obj.pose.position.y = tmp.pose.y;
      obj.pose.position.z = tmp.pose.z;
      obj.velocity.linear.x = tmp.velocity.x;
      obj.velocity.linear.y = tmp.velocity.y;
      obj.velocity.linear.z = 0.0;
      obj.pose.orientation.w = tmp.orientation.w;      
      obj.pose.orientation.x = tmp.orientation.x;
      obj.pose.orientation.y = tmp.orientation.y;
      obj.pose.orientation.z = tmp.orientation.z;
    }
#endif // GLOBAL_TRACK
    obj.header = output.header;
    obj.space_frame = output.header.frame_id;
    obj.dimensions.x = tmp.dimension.x;
    obj.dimensions.y = tmp.dimension.y;
    obj.dimensions.z = tmp.dimension.z;
    obj.label = Label2Classification[tmp.label];
    obj.score = tmp.score;

    // obj.header = input_header_;
    // obj.space_frame = "velodyne";
    obj.id = tmp.id;

    obj.angle = tmp.features.yaw;
    obj.color.r = tmp.color.r;
    obj.color.g = tmp.color.g;
    obj.color.b = tmp.color.b;
    obj.color.a = tmp.color.a;

    obj.valid = tmp.valid;
    obj.pose_reliable = tmp.pose_reliable;
    obj.velocity_reliable = tmp.velocity_reliable;
    obj.behavior_state = 0;
    
    output.objects.push_back(obj);
  }
  out_objs_.clear();
}


void LidarTrackRos::DumpResultText(autoware_msgs::DetectedObjectArray& detected_objects)
{
  std::ofstream outputfile(kitti_data_dir_, std::ofstream::out | std::ofstream::app);
  for (size_t i = 0; i < detected_objects.objects.size(); i++)
  {
    double yaw = tf::getYaw(detected_objects.objects[i].pose.orientation);

    yaw = yaw + 0.5 * M_PI;
    if(yaw > M_PI)
      yaw = yaw - 2 * M_PI;
    
    std::string classification = detected_objects.objects[i].label;
    //std::string classification = "Unknown";
    if (detected_objects.objects[i].label == "Car" ||  detected_objects.objects[i].label == "Truck") {
       classification = "Car";
    }
    //} else if (detected_objects.objects[i].label == "person" ) {
    //    classification = "Pedestrian";
    //} else if (detected_objects.objects[i].label == "Cyclist") {
    //    classification = "Cyclist";
    //}

    // KITTI tracking benchmark data format:
    // (frame_number,tracked_id, object type, truncation, occlusion, observation angle, x1,y1,x2,y2, h, w, l, cx, cy,
    // cz, yaw)
    // x1, y1, x2, y2 are for 2D bounding box.
    // h, w, l, are for height, width, length respectively
    // cx, cy, cz are for object centroid

    // Tracking benchmark is based on frame_number, tracked_id,
    // bounding box dimentions and object pose(centroid and orientation) from bird-eye view

        outputfile << std::to_string(detected_objects.header.seq) << " "
              << std::to_string(detected_objects.objects[i].id) << " "
              << classification.c_str() << " "
              << "-1" << " "
              << "-1" << " "
              << "-1" << " "
              << "-1 -1 -1 -1" << " "
              << std::to_string(detected_objects.objects[i].dimensions.z) << " "
              << std::to_string(detected_objects.objects[i].dimensions.y) << " "
              << std::to_string(detected_objects.objects[i].dimensions.x) << " "
              << std::to_string(0.06 - detected_objects.objects[i].pose.position.y) << " "
              << std::to_string(1.73 - detected_objects.objects[i].pose.position.z) << " "
              << std::to_string(detected_objects.objects[i].pose.position.x - 0.27) << " "
              << std::to_string(yaw) << " "
              << std::to_string(detected_objects.objects[i].score) << "\n";
  }
  frame_count_++;
}


