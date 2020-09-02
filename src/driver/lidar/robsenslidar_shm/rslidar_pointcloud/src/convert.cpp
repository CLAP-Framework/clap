/*
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2011 Jesse Vera
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2017 Robosense, Tony Zhang
 *  Copyright (C) 2020 Novauto, Yafei Sun
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file

    This class converts raw RSLIDAR 3D LIDAR packets to PointCloud2.

*/
#include "convert.h"
#include <pcl_conversions/pcl_conversions.h>
//for rsdriver
#include <rslidar_msgs/rslidarScan.h>
//for rsdriver end

namespace rslidar_pointcloud
{
  std::string model;
//for rsdriver
  static const unsigned int POINTS_ONE_CHANNEL_PER_SECOND = 18000;
  static const unsigned int BLOCKS_ONE_CHANNEL_PER_PKT = 12;
//for rsdriver end

/** @brief Constructor. */
Convert::Convert(ros::NodeHandle node, ros::NodeHandle private_nh) : 
    data_(new rslidar_rawdata::RawData()),
    node_(node)
{
  data_->loadConfigFile(node, private_nh);  // load lidar parameters     
  private_nh.param("model", model, std::string("RS32"));

  //communication mode: shm (shm_enable = true) or pub&sub(shm_enable = false)
  private_nh.param("shm_enable", shm_enable_, false);///
  ROS_INFO_STREAM("[cloud_node] shm_enable is : " << shm_enable_);
  // advertise output point cloud (before subscribing to input data)
  if(!shm_enable_){
    std::string output_points_topic;
    private_nh.param("output_points_topic", output_points_topic, std::string("rslidar_points"));
    output_ = node.advertise<sensor_msgs::PointCloud2>(output_points_topic, 10);
  }
  else{
    //for rviz temperarily
    std::string output_points_topic;
    private_nh.param("output_points_topic", output_points_topic, std::string("rslidar_points"));
    output_ = node.advertise<sensor_msgs::PointCloud2>(output_points_topic, 10);
   //for rviz temperarily
    int shm_key, shm_block_size;     //middle:200, left:201, right:202
    private_nh.param("shm_key", shm_key, 200);
    private_nh.param("shm_block_size", shm_block_size, 10);
    private_nh.param("shm_client_num", shm_client_num_, 1);
    int sem_proj_id[shm_client_num_];
    for(int i=0; i < shm_client_num_; i++){
      //信号量
      private_nh.param("sem_proj_id" + std::to_string(i), sem_proj_id[i], 100);
      ROS_INFO_STREAM("[cloud_node] sem project id is : " << sem_proj_id[i] <<", index is :" << i);
      sem_id_[i] = sem_object_[i].CreatSemid(sem_proj_id[i], 1);      //创建信号量
      if (sem_id_[i] == -1)
      {
        ROS_ERROR_STREAM("[cloud_node]semget failed and sem index is " << i << ".");
      }
      sem_object_[i].InitSem(sem_id_[i], 0, 1);           //0：当前信号集的哪一个信号量
    }

    ROS_INFO_STREAM("[cloud_node] shm key is : " << shm_key 
      <<", /nshm block size is :" << shm_block_size
      <<", /nshm client number is :" << shm_client_num_);
    shm_obj_point_ = shared_memory::ShmObject{shm_data_length__, shm_block_size, shm_key, shm_client_num_};
  }

  // raw packet output topic
  std::string output_packets_topic;
  private_nh.param("output_packets_topic", output_packets_topic, std::string("rslidar_packets"));
  msop_output_ = node.advertise<rslidar_msgs::rslidarScan>(output_packets_topic, 10);

  std::string output_difop_topic;
  private_nh.param("output_difop_topic", output_difop_topic, std::string("rslidar_packets_difop"));
  difop_output_ = node.advertise<rslidar_msgs::rslidarPacket>(output_difop_topic, 10);

  srv_ = boost::make_shared<dynamic_reconfigure::Server<rslidar_pointcloud::CloudNodeConfig> >(private_nh);
  dynamic_reconfigure::Server<rslidar_pointcloud::CloudNodeConfig>::CallbackType f;
  f = boost::bind(&Convert::callback, this, _1, _2);
  srv_->setCallback(f);

  ///////////////from rsdriver
  skip_num_ = 0;
  // use private node handle to get parameters
  private_nh.param("frame_id", config_.frame_id, std::string("rslidar"));
  
  std::string tf_prefix = tf::getPrefixParam(private_nh);
  ROS_DEBUG_STREAM("tf_prefix: " << tf_prefix);
  config_.frame_id = tf::resolve(tf_prefix, config_.frame_id);

  // get model name, validate string, determine packet rate
  private_nh.param("model", config_.model, std::string("RS16"));
  double packet_rate;  // packet frequency (Hz)
  std::string model_full_name;

  // product model
  if (config_.model == "RS16")
  {
    //for 0.18 degree horizontal angle resolution
    //packet_rate = 840;
    //for 0.2 degree horizontal angle resolution
    packet_rate = 750;
    model_full_name = "RS-LiDAR-16";
  }
  else if (config_.model == "RS32")
  {
    //for 0.18 degree horizontal angle resolution
    //packet_rate = 1690;
    //for 0.2 degree horizontal angle resolution
    packet_rate = 1500;       //150 packets per scan, 10 scan per second
    model_full_name = "RS-LiDAR-32";
  }
  else if (config_.model == "RSBPEARL")
  {
    packet_rate = 1500;
    model_full_name = "RSBPEARL";
  }
  else
  {
    ROS_ERROR_STREAM("[cloud_node] unknown LIDAR model: " << config_.model);
    packet_rate = 2600.0;
  }
  std::string deviceName(std::string("Robosense ") + model_full_name);

  private_nh.param("rpm", config_.rpm, 600.0);
  double frequency = (config_.rpm / 60.0);  // expected Hz rate

  // default number of packets for each scan is a single revolution
  // (fractions rounded up)

  int npackets = (int)ceil(packet_rate / frequency);    //1500/10 = 150
  private_nh.param("npackets", config_.npackets, npackets);   //defaut 150
  ROS_INFO_STREAM("[cloud_node] publishing " << config_.npackets << " packets per scan");

  std::string dump_file;
  private_nh.param("pcap", dump_file, std::string(""));

  int msop_udp_port;
  private_nh.param("msop_port", msop_udp_port, (int)MSOP_DATA_PORT_NUMBER);
  int difop_udp_port;
  private_nh.param("difop_port", difop_udp_port, (int)DIFOP_DATA_PORT_NUMBER);

  double cut_angle;
  private_nh.param("cut_angle", cut_angle, -0.01);
  if (cut_angle < 0.0)
  {
    ROS_INFO_STREAM("[cloud_node] Cut at specific angle feature deactivated.");
  }
  else if (cut_angle < 360)
  {
    ROS_INFO_STREAM("[cloud_node] Cut at specific angle feature activated. "
                    "Cutting rslidar points always at "
                    << cut_angle << " degree.");
  }
  else
  {
    ROS_ERROR_STREAM("[cloud_node] cut_angle parameter is out of range. Allowed range is "
                     << "between 0.0 and 360 negative values to deactivate this feature.");
    cut_angle = -0.01;
  }

  // Convert cut_angle from radian to one-hundredth degree,
  // which is used in rslidar packets
  config_.cut_angle = static_cast<int>(cut_angle * 100);

  // Initialize dynamic reconfigure
  /*srv_ = boost::make_shared<dynamic_reconfigure::Server<rslidar_driver::rslidarNodeConfig> >(private_nh);
  dynamic_reconfigure::Server<rslidar_driver::rslidarNodeConfig>::CallbackType f;
  f = boost::bind(&rslidarDriver::callback, this, _1, _2);
  srv_->setCallback(f);  // Set callback function und call initially    */
  /////////////////////wait to modify
  // initialize diagnostics
  diagnostics_.setHardwareID(deviceName);
  const double diag_freq = packet_rate / config_.npackets;
  diag_max_freq_ = diag_freq;
  diag_min_freq_ = diag_freq;
  // ROS_INFO("expected frequency: %.3f (Hz)", diag_freq);

  using namespace diagnostic_updater;
  diag_topic_.reset(new TopicDiagnostic("rslidar_packets", diagnostics_,
                                        FrequencyStatusParam(&diag_min_freq_, &diag_max_freq_, 0.1, 10),
                                        TimeStampStatusParam()));

  // open rslidar input device or file
  if (dump_file != "")  // have PCAP file?
  {
    // read data from packet capture file
    msop_input_.reset(new rslidar_pointcloud::InputPCAP(private_nh, msop_udp_port, packet_rate, dump_file));
    difop_input_.reset(new rslidar_pointcloud::InputPCAP(private_nh, difop_udp_port, packet_rate, dump_file));
  }
  else
  {
    // read data from live socket
    msop_input_.reset(new rslidar_pointcloud::InputSocket(private_nh, msop_udp_port));
    difop_input_.reset(new rslidar_pointcloud::InputSocket(private_nh, difop_udp_port));
  }

//"time_synchronization"
  private_nh.param("time_synchronization", time_synchronization_, false);///

  if (time_synchronization_)///
  {
    output_sync_ = node.advertise<sensor_msgs::TimeReference>("sync_header", 1);///
  }
  /////////////from rsdriver end   
}       //end of Convert() constructor

void Convert::callback(rslidar_pointcloud::CloudNodeConfig& config, uint32_t level)
{
//  ROS_INFO("[cloud_node] Reconfigure Request");  
  config_.time_offset = config.time_offset;   //just for rsdriver
}

/** @brief Callback for raw scan messages. */
void Convert::processScan(const rslidar_msgs::rslidarScan::ConstPtr& scanMsg)   
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr outPoints(new pcl::PointCloud<pcl::PointXYZI>);
  outPoints->header.stamp = pcl_conversions::toPCL(scanMsg->header).stamp;
  outPoints->header.frame_id = scanMsg->header.frame_id;
  outPoints->clear();
  if (model == "RS16")
  {
    outPoints->height = 16;
    outPoints->width = 24 * (int)scanMsg->packets.size();       //
    outPoints->is_dense = false;
    outPoints->resize(outPoints->height * outPoints->width);
  }
  else if (model == "RS32" || model == "RSBPEARL")
  {
    outPoints->height = 32;
    outPoints->width = 12 * (int)scanMsg->packets.size();   //12*150 = 1800
    outPoints->is_dense = false;
    outPoints->resize(outPoints->height * outPoints->width);      //32*1800 = 
  }

  // process each packet provided by the driver
  data_->block_num = 0;
  for (size_t i = 0; i < scanMsg->packets.size(); ++i)
  {
    data_->unpack(scanMsg->packets[i], outPoints);
  }

  if(shm_enable_ == false){
    sensor_msgs::PointCloud2 outMsg;
    pcl::toROSMsg(*outPoints, outMsg);
    output_.publish(outMsg);
  }
  else{
    //for rviz temperarily
    sensor_msgs::PointCloud2 outMsg;
    pcl::toROSMsg(*outPoints, outMsg);
    output_.publish(outMsg);
    //for rviz temperarily

    shmPublish(*outPoints);
  }
}     //end of processScan()

void Convert::shmPublish(const pcl::PointCloud<pcl::PointXYZI> &out_points){
    //serialize outMsg start    
    pcl::PCLPointCloud2 pcl_pc2;
    pcl::toPCLPointCloud2(out_points, pcl_pc2);
    rslidarPointStruct rslidar_point_to_struct{pcl_pc2};
    //信号量获取
    // ROS_WARN_STREAM("[SEM_TEST] shm_client_num_ is " << shm_client_num_);
    for(int i=0; i < shm_client_num_; i++){
      // ROS_WARN_STREAM("[SEM_TEST] sem_id_ is " << sem_id_[i] 
      //   << ", and index is " << i);
      sem_object_[i].P(sem_id_[i], 0, -1);
    }

    shm_data_point_=shm_obj_point_.require_data();//要求数据          unsigned char*
    if(shm_data_point_!=NULL)
    {
        std::memcpy(shm_data_point_+MAX_CLIENT_NUM, &rslidar_point_to_struct, shm_data_length__); 
        shm_obj_point_.update_shm(shm_data_point_);//更新数据，可被读取
    }
    else{
        ROS_INFO("[cloud_node] shm data pointer is NULL.");
    }
    //信号量返还，写SHM后
    for(int i=0; i < shm_client_num_; i++){
      sem_object_[i].V(sem_id_[i], 0, 1);
    }    
}   //end of shmPublish()

void Convert::msop_thread(void){
  if (time_synchronization_)
  {
    skip_num_sub_ = node_.subscribe<std_msgs::Int32>("skippackets_num", 1, &Convert::skipNumCallback,///
                                                    (Convert*)this, ros::TransportHints().tcpNoDelay(true));///move it to difop_thread, note to lock
  }       //just subscribe an int32
  while (ros::ok() && poll())
  {    
    ros::spinOnce();
  }
}

////from rsdriver
/** poll the device
 *
 *  @returns true unless end of file reached
 */
bool Convert::poll(void)        ///
{  // Allocate a new shared pointer for zero-copy sharing with other nodelets.
  rslidar_msgs::rslidarScanPtr scan(new rslidar_msgs::rslidarScan);
  // Since the rslidar delivers data at a very high rate, keep
  // reading and publishing scans as fast as possible.
  if (config_.cut_angle >= 0)  // Cut at specific angle feature enabled
  {
    scan->packets.reserve(config_.npackets);        //150 packets for RS32
    rslidar_msgs::rslidarPacket tmp_packet;
    while (true)
    {
      while (true)
      {
        int rc = msop_input_->getPacket(&tmp_packet, config_.time_offset);
        if (rc == 0)
          break;  // got a full packet?
        if (rc < 0)
          return false;  // end of file reached?
      }
      scan->packets.push_back(tmp_packet);

      static int ANGLE_HEAD = -36001;  // note: cannot be set to -1, or stack smashing
      static int last_azimuth = ANGLE_HEAD;

      int azimuth = 256 * tmp_packet.data[44] + tmp_packet.data[45];
      // int azimuth = *( (u_int16_t*) (&tmp_packet.data[azimuth_data_pos]));

      // Handle overflow 35999->0
      if (azimuth < last_azimuth)
      {
        last_azimuth -= 36000;
      }
      // Check if currently passing cut angle
      if (last_azimuth != ANGLE_HEAD && last_azimuth < config_.cut_angle && azimuth >= config_.cut_angle)
      {
        last_azimuth = azimuth;
        break;  // Cut angle passed, one full revolution collected
      }
      last_azimuth = azimuth;
    }
  }
  else  // standard behaviour
  {
    if (difop_input_->getUpdateFlag())
    {
      int packets_rate = ceil(POINTS_ONE_CHANNEL_PER_SECOND/BLOCKS_ONE_CHANNEL_PER_PKT);
      int mode = difop_input_->getReturnMode();
      if (config_.model == "RS16" && (mode == 1 || mode == 2))
      {
        packets_rate = ceil(packets_rate/2);
      }
      else if ((config_.model == "RS32" || config_.model == "RSBPEARL") && (mode == 0))
      {
        packets_rate = packets_rate*2;
      }
      config_.rpm = difop_input_->getRpm();
      config_.npackets = ceil(packets_rate*60/config_.rpm);

      difop_input_->clearUpdateFlag();

      ROS_INFO_STREAM("[cloud_node] update npackets. rpm: "<<config_.rpm<<", npkts: "<<config_.npackets);
    }
    scan->packets.resize(config_.npackets);       //
    // use in standard behaviour only
    while (skip_num_)
    {
      while (true)
      {
        // keep reading until full packet received
        int rc = msop_input_->getPacket(&scan->packets[0], config_.time_offset);
        if (rc == 0)
          break;  // got a full packet?
        if (rc < 0)
          return false;  // end of file reached?
      }
      --skip_num_;
    }

    for (int i = 0; i < config_.npackets; ++i)
    {
      while (true)
      {
        // keep reading until full packet received
        int rc = msop_input_->getPacket(&scan->packets[i], config_.time_offset);
        if (rc == 0)
          break;  // got a full packet?
        if (rc < 0)
          return false;  // end of file reached?
      }
    }

    if (time_synchronization_)
    {
      sensor_msgs::TimeReference sync_header;
      // it is already the msop msg
      // if (pkt->data[0] == 0x55 && pkt->data[1] == 0xaa && pkt->data[2] == 0x05 && pkt->data[3] == 0x0a)
      // use the first packets
      rslidar_msgs::rslidarPacket pkt = scan->packets[0];
      struct tm stm;
      memset(&stm, 0, sizeof(stm));
      stm.tm_year = (int)pkt.data[20] + 100;
      stm.tm_mon  = (int)pkt.data[21] - 1;
      stm.tm_mday = (int)pkt.data[22];
      stm.tm_hour = (int)pkt.data[23];
      stm.tm_min  = (int)pkt.data[24];
      stm.tm_sec  = (int)pkt.data[25];
      double stamp_double = mktime(&stm) + 0.001 * (256 * pkt.data[26] + pkt.data[27]) +
                            0.000001 * (256 * pkt.data[28] + pkt.data[29]);
      sync_header.header.stamp = ros::Time(stamp_double);
      output_sync_.publish(sync_header);
    }
  }     //end of else   
  // publish message using time of last packet read
//  ROS_DEBUG("[cloud_node] Publishing a full rslidar scan.");
  scan->header.stamp = scan->packets.back().stamp;
  scan->header.frame_id = config_.frame_id;
  
  msop_output_.publish(scan);
  processScan(scan);        //rslidar_msgs::rslidarScanPtr scan(new rslidar_msgs::rslidarScan);
                                                  //Convert::processScan(const rslidar_msgs::rslidarScan::ConstPtr& scanMsg)
  // notify diagnostics that a message has been published, updating its status
  diag_topic_->tick(scan->header.stamp);
  diagnostics_.update();
  return true;
}     //end of poll(void)

void Convert::difop_thread(void){      //new thread here , deal with failure.
  rslidar_msgs::rslidarPacket* difop_packet_ptr(new rslidar_msgs::rslidarPacket);
  while (ros::ok() &&difopPoll(difop_packet_ptr ))         
  {
    data_->processDifop(difop_packet_ptr);        //TODO: make sure if need mutex
    ros::spinOnce();
  }    
}

bool Convert::difopPoll(rslidar_msgs::rslidarPacket* difop_packet_ptr)       ///
{
    // keep reading
    rslidar_msgs::rslidarPacket difop_packet_msg;
    int rc = difop_input_->getPacket(&difop_packet_msg, config_.time_offset);
    if (rc == 0)
    {
      *difop_packet_ptr = difop_packet_msg;
      difop_output_.publish(difop_packet_msg);
      return true;
    }
    if (rc < 0)     // file empty or read_once. For socket always cycle.
      ROS_WARN("[cloud_node] Rslidar difop_poll() timeout");
      return false;  // end of file reached?
}   //end of difopPoll(void)

// add for time synchronization
void Convert::skipNumCallback(const std_msgs::Int32::ConstPtr& skip_num)
{
  // std::cout << "Enter skipNumCallback: " << skip_num->data << std::endl;
  skip_num_ = skip_num->data;
}
////from rslidar end

}  // namespace rslidar_pointcloud
