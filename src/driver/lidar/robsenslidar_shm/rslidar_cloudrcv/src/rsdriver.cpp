/*
 *  Copyright (C) 2007 Austin Robot Technology, Patrick Beeson
 *  Copyright (C) 2009-2012 Austin Robot Technology, Jack O'Quin
 *	Copyright (C) 2017 Robosense, Tony Zhang
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  ROS driver implementation for the RILIDAR 3D LIDARs
 */
#include "rsdriver.h"
#include <rslidar_msgs/rslidarScan.h>
#include <unistd.h>


namespace rslidar_driver
{
  static const unsigned int POINTS_ONE_CHANNEL_PER_SECOND = 18000;
  static const unsigned int BLOCKS_ONE_CHANNEL_PER_PKT = 12;
  std::string model;

rslidarDriver::rslidarDriver(ros::NodeHandle node, ros::NodeHandle private_nh)
{    
  private_nh.param("shm_enable", shm_enable_rcv__, false);
  ROS_INFO_STREAM( "[RSLIDAR_POINTRCV] shm_enable_rcv__ is: " << shm_enable_rcv__);

  private_nh.param("model", model, std::string("RS16"));

  pid_ = getpid();		
  ROS_INFO_STREAM( model<< " rslidar-rsdriver process id is: " << static_cast<int>(pid_)); 

  // Initialize dynamic reconfigure
  srv_ = boost::make_shared<dynamic_reconfigure::Server<rslidar_driver::rslidarNodeConfig> >(private_nh);
  dynamic_reconfigure::Server<rslidar_driver::rslidarNodeConfig>::CallbackType f;
  f = boost::bind(&rslidarDriver::callback, this, _1, _2);
  srv_->setCallback(f);  // Set callback function und call initially

  // advertise output point cloud (before subscribing to input data)  
  msg_nsec_last_ = 0;

    if(shm_enable_rcv__){
      //信号量
      sem_id_ = sem_object_rec_.GetSemid(100, 0);      //打开信号量，0即可
      if (sem_id_ == -1)
      {
        ROS_ERROR("[REC_node] semget open sem failed.");
      } 
      ROS_INFO("[REC_node] semget open sem success.");

    //for feedback part
      output_ = node.advertise<sensor_msgs::PointCloud2>("rslidar_points_SHM", 10);
      boost::shared_ptr<boost::thread> shm_thread = 
          boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&rslidarDriver::recvAndSend, this)));
    }else{
      output_ros_ = node.advertise<sensor_msgs::PointCloud2>("rslidar_points_ROS", 10);
      pcl_sub_ = node.subscribe("rslidar_inpoints", 10, &rslidarDriver::subPoint,
                                      (rslidarDriver*)this, ros::TransportHints().tcpNoDelay(true));
    }
}   //end of constructor

void rslidarDriver::writeFile() {
  //sample per 100ms
    if(msg_nsec_ != msg_nsec_last_) {
      std::string value_string = std::to_string(msg_sec_) + ", " + std::to_string(msg_nsec_)
                           + ", " + std::to_string(time_now_) + "\n";
      char buf[50]={0};
      std::strcpy(buf, value_string.c_str());
      if(write(fd_,buf,std::strlen(buf))==-1)// write we need the file unistd.h strlen need the file cstring
      {
        ROS_ERROR_STREAM("[rsdriver_node] - write file error with the code :" << errno);
        return;
      }
      msg_nsec_last_ = msg_nsec_;
      //usleep(100000);
    }   //end of if
}

void rslidarDriver::recvAndSend(){
  //sleep(2);
  rslidarPointStruct struct_of_point;
  bool sem_exit_flag = sem_object_rec_.IfSemExist(100);
  while(sem_exit_flag){           //not check every cycle
    //思路，信号量有了就读取，根据write index决定读取位置，write index - 1帧
    sem_object_rec_.P(sem_id_, 0, -1);    //信号量，读SHM前
   
      // read_data_ = shm_obj_point_.get_data_for_read_with_sem();    //unsigned char*
      read_data_ = shm_obj_point_.get_data_for_read_with_sem(1);    //unsigned char*
          //ROS_INFO("[rsdriver_node] read_data_ first data is : %d!", *read_data_[1]);
      if(read_data_!=NULL)//表示已经获取到可读数据
      {
          //ROS_INFO("[rsdriver_node] get data from SHM!");
          // std::memcpy(&struct_of_point, read_data_+1, 1843340);        //RS32
          std::memcpy(&struct_of_point, read_data_+20, 1843340);        //RS32
          // read_data_[0]--;//读取完成标志位
          read_data_[0] = 0;//读取完成标志位
         // time_now_ =ros::Time::now().toSec();    //002. 26ms 延时测试     
          //ROS_INFO("[REC_node] DATA success!!!");
          sensor_msgs::PointCloud2 point_msg = struct_of_point.get_point_msg();  
          output_.publish(point_msg);
          //msg_sec_ = (int)point_msg.header.stamp.sec;
          //msg_nsec_ = (int)point_msg.header.stamp.nsec;            
          //writeFile();       //latency test
          usleep(70000);     //wait for next frame.
      }
    sem_object_rec_.V(sem_id_, 0, 1);

    usleep(2000);     //20Hz
  }     //end of while((sem_object_rec_.IfSemExist(100)))
}   //end of recvAndSend()

void rslidarDriver::subPoint(const sensor_msgs::PointCloud2& point_msg){
    output_ros_.publish(point_msg);
}

void rslidarDriver::callback(rslidar_driver::rslidarNodeConfig& config, uint32_t level)
{  
}

}   //end of namespace rslidar_driver

