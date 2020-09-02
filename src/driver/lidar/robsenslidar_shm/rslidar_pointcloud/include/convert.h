/* -*- mode: C++ -*- */
/*
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2011 Jesse Vera
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2017 Robosense, Tony Zhang
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file

    This class converts raw Robosense 3D LIDAR packets to PointCloud2.

*/
#ifndef _CONVERT_H_
#define _CONVERT_H_

#include <sensor_msgs/PointCloud2.h>
#include <dynamic_reconfigure/server.h>
#include <rslidar_pointcloud/CloudNodeConfig.h>
#include "rawdata.h"

//for SHM
#include "point_struct.h"
#include "shm_object.h"
#include "sem_object.h"
//boost serialization
#include "boost/archive/binary_oarchive.hpp"
//#include "boost/archive/binary_iarchive.hpp"

#include <boost/serialization/list.hpp>
#include <boost/serialization/string.hpp>
#include <boost/serialization/version.hpp>
#include <boost/serialization/split_member.hpp> 
#include "boost/serialization/serialization.hpp"
#include <boost/serialization/export.hpp>
#include "boost/foreach.hpp"
#include "boost/any.hpp"
#include <boost/serialization/vector.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>

//for rsdriver
#include <string>
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Int32.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
//#include <rslidar_pointcloud/rslidarNodeConfig.h>
#include <pcl/point_types.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include "input.h"
//for rsdriver end

namespace rslidar_pointcloud
{
class Convert
{
public:
  Convert(ros::NodeHandle node, ros::NodeHandle private_nh);

  ~Convert()
  {
    // if(shm_enable_){
      // ROS_WARN_STREAM("[CLOUD] shm_client_num_ is " << shm_client_num_);
      // for(int i=0; i < shm_client_num_; i++){
        // sem_object_[i].Destory(sem_id_[i]);
        printf("[CLOUD] shm_client_num_ is %d\n",shm_client_num_);
        sem_object_[0].Destory(sem_id_[0]);
        sem_object_[1].Destory(sem_id_[1]);
        sem_object_[2].Destory(sem_id_[2]);
        printf("[CLOUD]deconstructor.");
      // }      
      shm_obj_point_.write_release();
    // }
  }
  //from rsdriver
  //bool poll(void);
  void msop_thread(void);
  bool poll(void);
  void difop_thread(void);
  bool difopPoll(rslidar_msgs::rslidarPacket* difop_packet_p);
  //from rsdriver end   

private:
  void callback(rslidar_pointcloud::CloudNodeConfig& config, uint32_t level);

  void processScan(const rslidar_msgs::rslidarScan::ConstPtr& scanMsg);

  /// Pointer to dynamic reconfigure service srv_
  boost::shared_ptr<dynamic_reconfigure::Server<rslidar_pointcloud::CloudNodeConfig> > srv_;

  boost::shared_ptr<rslidar_rawdata::RawData> data_;
  ros::Subscriber rslidar_scan_;
  ros::Publisher output_;

  ////from rsdriver
  /// Callback for dynamic reconfigure
/////  void callback(rslidar_driver::rslidarNodeConfig& config, uint32_t level);
  /// Callback for skip num for time synchronization
  void skipNumCallback(const std_msgs::Int32::ConstPtr& skip_num);

  /// Pointer to dynamic reconfigure service srv_
  /////boost::shared_ptr<dynamic_reconfigure::Server<rslidar_driver::rslidarNodeConfig> > srv_;  //TODO:

  // configuration parameters
  struct
  {
    std::string frame_id;  ///< tf frame ID
    std::string model;     ///< device model name
    int npackets;          ///< number of packets to collect
    double rpm;            ///< device rotation rate (RPMs)
    double time_offset;    ///< time in seconds added to each  time stamp
    int cut_angle;
  } config_;

  boost::shared_ptr<Input> msop_input_;
  boost::shared_ptr<Input> difop_input_;

  ros::Publisher msop_output_;
  ros::Publisher difop_output_;
  ros::Publisher output_sync_;
  // Converter convtor_;
  /** diagnostics updater */
  diagnostic_updater::Updater diagnostics_;
  double diag_min_freq_;
  double diag_max_freq_;
  boost::shared_ptr<diagnostic_updater::TopicDiagnostic> diag_topic_;

  //boost::shared_ptr<boost::thread> difop_thread_;   ///

  // add for time synchronization
  bool time_synchronization_;
  uint32_t skip_num_;
  ros::Subscriber skip_num_sub_;
  ////from rsdriver end 
  ros::NodeHandle node_;

  //for SHM
  bool shm_enable_;
  shared_memory::ShmObject shm_obj_point_;
  unsigned char* shm_data_point_;
  int shm_client_num_;
  //SEM
  shared_memory::SemObject sem_object_[MAX_CLIENT_NUM];
  int sem_id_[MAX_CLIENT_NUM];

  void shmPublish(const pcl::PointCloud<pcl::PointXYZI> &out_points);
  void printOutToTxt(const sensor_msgs::PointCloud2& outMsg);  
};

}  // namespace rslidar_pointcloud
#endif
