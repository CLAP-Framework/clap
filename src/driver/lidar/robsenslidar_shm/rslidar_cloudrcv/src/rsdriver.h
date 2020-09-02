/* -*- mode: C++ -*- */
/*
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *	Copyright (C) 2017 Robosense, Tony Zhang
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  ROS driver interface for the RSLIDAR 3D LIDARs
 */
#ifndef _RSDRIVER_H_
#define _RSDRIVER_H_

#include <string>
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Int32.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <dynamic_reconfigure/server.h>
#include <rslidar_driver/rslidarNodeConfig.h>
#include <pcl/point_types.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <boost/thread/thread.hpp>
#include "input.h"


//headers in Autowae Health Checker
//#include <health_checker/node_status_publisher.h>


#include "shm_object.h"
#include "sem_object.h"
//boost serialization
#include <boost/serialization/list.hpp>
#include <boost/serialization/string.hpp>
#include <boost/serialization/version.hpp>
#include <boost/serialization/split_member.hpp> 
#include "boost/serialization/serialization.hpp"
#include "boost/archive/binary_oarchive.hpp"
#include "boost/archive/binary_iarchive.hpp"
#include <boost/serialization/export.hpp>
#include "boost/foreach.hpp"
#include "boost/any.hpp"
#include <boost/serialization/vector.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>


namespace rslidar_driver
{
  bool shm_enable_rcv__ = false;
  const int rslidar_type_16_32 = false; //false: RS32, true: RS16


class rslidarDriver
{
public:
  /**
 * @brief rslidarDriver
 * @param node          raw packet output topic
 * @param private_nh    通过这个节点传参数
 */
  rslidarDriver(ros::NodeHandle node, ros::NodeHandle private_nh);

  ~rslidarDriver()
  {
    close(fd_); 
    if(shm_enable_rcv__){
    //for scanMsg
    //sem_object_rec_.Destory(sem_id_);   //reader no need to destroy
    shm_obj_point_.read_release();
    ROS_INFO("[rsdriver_node] Free SHM!!!");}
    //if(read_data_!=NULL) free(read_data_);  

  }
 // bool poll(void);
 // void difopPoll(void);

private:
  /// Callback for dynamic reconfigure
  void callback(rslidar_driver::rslidarNodeConfig& config, uint32_t level);

  /// Pointer to dynamic reconfigure service srv_
  boost::shared_ptr<dynamic_reconfigure::Server<rslidar_driver::rslidarNodeConfig> > srv_;

  //std::shared_ptr<health_checker::NodeStatusPublisher> node_status_pub_ptr_;
  pid_t pid_;
  // configuration parameters

    ros::Publisher output_ros_;
    ros::Publisher output_;
// #ifdef _SHM_  
    //
    ShmObject shm_obj_point_{200};
    unsigned char *read_data_;
    void recvAndSend();
// #else
    //for test
    ros::Subscriber pcl_sub_;
    //  ros::Subscriber pcl_sub_left_;
    // ros::Subscriber pcl_sub_right_;
    void subPoint(const sensor_msgs::PointCloud2& point_msg);
// #endif 
    ros::Publisher output_sync_;
    ros::Publisher msop_output_;
    ros::Publisher difop_output_;

  
  // Converter convtor_;
  /** diagnostics updater */
  diagnostic_updater::Updater diagnostics_;
  double diag_min_freq_;
  double diag_max_freq_;

  // add for time synchronization
  bool time_synchronization_;
  uint32_t skip_num_;

// for latency test
  int msg_sec_;
  int msg_nsec_;
  int msg_nsec_last_;
  double time_now_;
  int fd_;
  void writeFile();
  //信号量
  // #ifdef _SHM_
  shared_memory::SemObject sem_object_rec_;
  int sem_id_;
  // #endif
};



struct rslidarPointStruct
{
  //for header
  uint32_t seq_;
  uint64_t stamp_;
  char frame_id_[8] = {'0'};
  //end of header
  uint32_t height_;
  uint32_t width_;
  uint8_t fields_[96];     //(11 name + 4+1+4 =)20 * 4 =80  (32added)
  uint8_t is_bigendian_;
  uint8_t is_bigendian_blank_[3] = {0};
  uint32_t point_step_;
  uint32_t row_step_;
  uint8_t data_[1843200] = {0};
  uint8_t is_dense_;
  uint8_t is_dense_blank_[3] = {0};

  rslidarPointStruct(){}
	rslidarPointStruct(const sensor_msgs::PointCloud2& pointMsg){
    //header start
    seq_ = pointMsg.header.seq;
    stamp_ = pointMsg.header.stamp.toNSec() / 1000ull; 

    int id_length = pointMsg.header.frame_id.size();
    int min_length = std::min(8, id_length);
    for(int i=0;i<min_length;i++)
        frame_id_[i] = pointMsg.header.frame_id[i];
    // header end

    height_ = pointMsg.height;
    width_ = pointMsg.width;

    int i = 0;
    int j = 0;
    //fields start
    do{
      int name_length = pointMsg.fields[i].name.size();
      int min_length = std::min(11, name_length);
      j = 0;
      do{
        fields_[i * 20 + j] = (uint8_t)(pointMsg.fields[i].name[j]);
        j++;
      }while(j < min_length);
      std::memcpy(&fields_[i * 20 + 11], &pointMsg.fields[i].offset, 4);
      fields_[i * 20 + 15] = pointMsg.fields[i].datatype;
      std::memcpy(&fields_[i * 20 + 16], &pointMsg.fields[i].count, 4);
      i++;
    }while(i<4);
    //ROS_INFO("[cloud_node] fields number added ALREADY!!!!!");
    //fields end
    
    is_bigendian_ = pointMsg.is_bigendian;
    point_step_ = pointMsg.point_step;
    row_step_ = pointMsg.row_step;

    i = 0;
    do{
        data_[i] = pointMsg.data[i];
        i++;
    }while( i < pointMsg.data.size() );       //150*
    //std::memcpy(&data_, &pointMsg.data, 1843200);

    is_dense_ = pointMsg.is_dense;
  }     //end of constructor 

rslidarPointStruct(const pcl::PCLPointCloud2& pc2){
    // header end
    seq_ = pc2.header.seq;
    stamp_ = pc2.header.stamp;
    int id_length = pc2.header.frame_id.size();
    int min_length = std::min(8, id_length);
    std::memcpy(&frame_id_, pc2.header.frame_id.data(), min_length);
    // header end
    height_ = pc2.height;
    width_ = pc2.width;
    int i = 0;
    //fields start
    do{
      int name_length = pc2.fields[i].name.size();
      int min_length = std::min(12, name_length);
      std::memcpy(&fields_[i * 24], pc2.fields[i].name.data(), min_length);
      std::memcpy(&fields_[i * 24 + 12], &pc2.fields[i].offset, 12);       //12 + 4+4+4 = 24
      i++;
    }while(i<pc2.fields.size());
    //fields end    
    is_bigendian_ = pc2.is_bigendian;
    point_step_ = pc2.point_step;
    row_step_ = pc2.row_step;
    if(!pc2.data.empty()){
      std::memcpy(&data_, &pc2.data[0], pc2.data.size());
    }
    is_dense_ = pc2.is_dense;
}     //end of constructor(pc2) 


  pcl::PCLPointCloud2 get_pcl_point_cloud2(){
      pcl::PCLPointCloud2 pc2;
      //header start
      pc2.header.seq = seq_;      
      pc2.header.stamp = stamp_;    //   = ros::Time(sec, nsec); 
      pc2.header.frame_id = frame_id_;
      //header end

      pc2.height = height_;
      pc2.width = width_;

    int i = 0;
    int j;
    //fields start
      pc2.fields.resize(0);
      ::pcl::PCLPointField field;
      do{
        j = 0;
        do{
          if(fields_[i * 24 + j]){
            field.name += (char)(fields_[i * 24 + j]);
          }
          j++;
        }while(j < 12);
        std::memcpy(&field.offset, &fields_[i * 24 + 12], 12);
        pc2.fields.push_back(field); 
        field.name = "";
        i++;
      }while(i<4);
      //fields end    

      pc2.is_bigendian = is_bigendian_;
      pc2.point_step = point_step_;
      pc2.row_step = row_step_;

      pc2.data.reserve(1843200);
      pc2.data.assign(&data_[0], &data_[1843200]);   //end address is 1843200, not 1843199

      pc2.is_dense = is_dense_;
      return pc2;
  } 

  sensor_msgs::PointCloud2 get_point_msg(){
      sensor_msgs::PointCloud2 pointMsg;
      //header start
      pointMsg.header.seq = seq_;      
      pointMsg.header.stamp.fromNSec(stamp_ * 1000ull);    //   = ros::Time(sec, nsec); 
      pointMsg.header.frame_id = frame_id_;
      //header end

      pointMsg.height = height_;
      pointMsg.width = width_;

    int i = 0;
    int j;
    //fields start
      pointMsg.fields.resize(0);
      sensor_msgs::PointField_<std::allocator<void>> field;
      do{
        j = 0;
        do{
          if(fields_[i * 24 + j]){
            field.name += (char)(fields_[i * 24 + j]);
          }
          j++;
        }while(j < 12);
        std::memcpy(&field.offset, &fields_[i * 24 + 12], 12);
        pointMsg.fields.push_back(field); 
        field.name = "";
        i++;
      }while(i<4);
      //fields end    

      pointMsg.is_bigendian = is_bigendian_;
      pointMsg.point_step = point_step_;
      pointMsg.row_step = row_step_;

      pointMsg.data.reserve(1843200);
      pointMsg.data.assign(&data_[0], &data_[1843200]);   //end address is 1843200, not 1843199

      pointMsg.is_dense = is_dense_;
      return pointMsg;
  }
};      // end of class rslidarPointStruct  


}  // namespace rslidar_driver
#endif
