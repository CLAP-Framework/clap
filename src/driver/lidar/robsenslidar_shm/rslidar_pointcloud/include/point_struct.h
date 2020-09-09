/**
 * @file point_struct.h
 * @brief pointCloud2 message convert to this struct for SHM communication.
 *
 * @author yafei.sun@novauto.com.cn
 * @version 0.0.1
 * @date 2020-03-13
 */

#ifndef _POINTSTRUCT_H_
#define _POINTSTRUCT_H_

#include <string>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include "boost/serialization/serialization.hpp"
namespace rslidar_pointcloud
{
const int RS32_point_data_size__ = 1843200;
const int RS16_point_data_size__ = 1843200;
const int point_data_size__ = RS32_point_data_size__;
const int shm_data_length__ = point_data_size__ + 140;

class rslidarPointStruct
{
public:
    rslidarPointStruct(){}
    rslidarPointStruct(const sensor_msgs::PointCloud2& pointMsg);
    rslidarPointStruct(const pcl::PCLPointCloud2& pc2);
    ~rslidarPointStruct(){} 
    
    pcl::PCLPointCloud2 get_pcl_point_cloud2();    
    sensor_msgs::PointCloud2 get_point_msg();

private:
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
    uint8_t data_[point_data_size__] = {0};
    uint8_t is_dense_;
    uint8_t is_dense_blank_[3] = {0};
};      // end of class rslidarPointStruct  

}  // namespace rslidar_pointcloud

#endif    //end of _POINTSTRUCT_H_
