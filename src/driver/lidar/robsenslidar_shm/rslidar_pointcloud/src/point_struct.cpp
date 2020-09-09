/**
 * @file point_struct.cpp
 * @brief pointCloud2 message convert to this struct for SHM communication.
 *
 * @author yafei.sun@novauto.com.cn
 * @version 0.0.1
 * @date 2020-03-13
 */
#include "point_struct.h"

namespace rslidar_pointcloud
{
  rslidarPointStruct::rslidarPointStruct(const sensor_msgs::PointCloud2& pointMsg){
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
    //std::memcpy(&data_, &pointMsg.data, point_data_size__);

    is_dense_ = pointMsg.is_dense;
  }     //end of constructor 

  rslidarPointStruct::rslidarPointStruct(const pcl::PCLPointCloud2& pc2){
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

  pcl::PCLPointCloud2 rslidarPointStruct::get_pcl_point_cloud2(){
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

    pc2.data.reserve(point_data_size__);
    pc2.data.assign(&data_[0], &data_[point_data_size__]);  

    pc2.is_dense = is_dense_;
    return pc2;
  }

  sensor_msgs::PointCloud2 rslidarPointStruct::get_point_msg(){
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

    pointMsg.data.reserve(point_data_size__);
    pointMsg.data.assign(&data_[0], &data_[point_data_size__]);   

    pointMsg.is_dense = is_dense_;
    return pointMsg;
  }
}   //end of namespace rslidar_driver

