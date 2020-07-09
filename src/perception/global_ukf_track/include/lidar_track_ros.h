// #pragma once
#ifndef _LIDAR_TRACK_ROS_H_
#define _LIDAR_TRACK_ROS_H_

#include "lidar_track.h"
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

#include "autoware_msgs/DetectedObjectArray.h"
#include <ros/ros.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <Eigen/Dense>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#define __APP_NAME__ "global_ukf_track"


class LidarTrackRos : public ele::LidarTrack {
public:
    LidarTrackRos() ;
    ~LidarTrackRos() ;

    void Run();
    void objectsCallback(const autoware_msgs::DetectedObjectArray& input);
    void navCallback(const sensor_msgs::NavSatFix& input);
    void imuCallback(const sensor_msgs::Imu& input);
    void odmCallback(const nav_msgs::Odometry& input);
    void syncCallback(const nav_msgs::Odometry::ConstPtr& pOdom, 
        const autoware_msgs::DetectedObjectArray::ConstPtr& pObj);
private:
    void SetDetectedObjects(const autoware_msgs::DetectedObjectArray& input);
    void GetTrackObjects(autoware_msgs::DetectedObjectArray& output);
    void DumpResultText(autoware_msgs::DetectedObjectArray& detected_objects);
private:

    // whether if benchmarking tracking result
    bool is_benchmark_;
    std::string kitti_data_dir_;
    std::string input_obj_topic_;
    std::string input_nav_topic_;
    std::string input_imu_topic_;
    std::string input_odm_topic_;
    
    std::string output_topic_;
    std::string output_zzz_topic_;

    bool global_output_;
    ros::NodeHandle node_handle_;
    ros::NodeHandle private_nh_;
    ros::Subscriber sub_detected_array_;
    ros::Subscriber sub_nav_;
    ros::Subscriber sub_imu_;
    ros::Subscriber sub_odm_;
    ros::Publisher  pub_object_array_;
    ros::Publisher  pub_zzz_object_array_;

    typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, 
        autoware_msgs::DetectedObjectArray> ObjectsSyncPolicy;
    message_filters::Subscriber<nav_msgs::Odometry>*    mf_sub_detected_array_;       
    message_filters::Subscriber<autoware_msgs::DetectedObjectArray>*    mf_sub_odm_; 
    message_filters::Synchronizer<ObjectsSyncPolicy>*   sync_;

    std_msgs::Header input_header_; 
    std::string space_frame_;
    autoware_msgs::DetectedObject   input_object_;
    sensor_msgs::NavSatFix          input_nav_;
    sensor_msgs::Imu                input_imu_;
    nav_msgs::Odometry              input_odm_;
    unsigned int                    frame_count_;
    Eigen::Matrix3d                 lidar2imu_rotation_;
    Eigen::Vector3d                 lidar2imu_translation_;
};



#endif // _LIDAR_TRACK_ROS_H_
