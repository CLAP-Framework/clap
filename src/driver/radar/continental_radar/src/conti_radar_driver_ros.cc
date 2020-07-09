
#include <cstdlib>
#include <string>
#include <sstream>
#include <cstdint>
#include "continental_radar/conti_radar_driver_ros.h"

#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include "visualization_msgs/MarkerArray.h"
#include "continental_radar/common/boost_udp.h"
#include "continental_radar/proto/conti_radar.h"
#include "continental_radar/proto/conti_radar_conf.h"
#include "continental_radar/frame/canet/canet.h"
#include "continental_radar/protocol/const_vars.h"
#include "continental_radar/protocol/radar_config_200.h"
#include "continental_radar/protocol/radar_state_201.h"
#include "continental_radar/protocol/motion_input_speed_300.h"
#include "continental_radar/protocol/motion_input_yawrate_301.h"
#include "continental_radar/protocol/cluster_list_status_600.h"
#include "continental_radar/protocol/object_list_status_60a.h"
#include "continental_radar/protocol/object_general_info_60b.h"
#include "continental_radar/protocol/object_quality_info_60c.h"
#include "continental_radar/protocol/object_extended_info_60d.h"
#include "continental_radar/protocol/cluster_general_info_701.h"
#include "continental_radar/protocol/cluster_quality_info_702.h"
#include "continental_radar_msgs/RadarMsg.h"
#include "continental_radar/common/param.h"

namespace drivers {

void ContiRadarDriverRos::init(const ros::NodeHandle* parent) 
{
  //
  // ros::NodeHandle nh2("~");
  // nh2.param("portHost", host_port_, 4001);
  // nh2.param("portTarget", target_port_, 4001);
  // nh2.param<std::string>("frameID", frame_id_, "continental_radar");
  // nh2.param<std::string>("ipHost", host_ip_, "192.168.110.1");
  // nh2.param<std::string>("ipTarget", target_ip_, "192.168.110.101");
  // nh2.param<std::string>("topicName", topic_name_, "RadarMsg");
  
  nh_ = const_cast<ros::NodeHandle*>(parent);
  host_ip_ = getParam < std::string >("continental_radar/Host_Addr", "192.168.0.1");
  host_port_ = getParam < int > ("continental_radar/Host_Port", 4004);
  target_ip_ = getParam < std::string >("continental_radar/Target_Addr", "192.168.0.2");
  target_port_ = getParam < int >("continental_radar/Target_Port", 4004);
  topic_name_ = getParam < std::string>("continental_radar/Topic_Name", "RadarMsg");
  frame_id_ = getParam < std::string> ("continental_radar/Frame_ID", "continental_radar");

 // node_status_pub_ptr_ = std::make_shared<health_checker::NodeStatusPublisher>(*nh_);
  //node_status_pub_ptr_->ENABLE();

  boost::asio::io_service io_service;
  common::BoostUdp* boost_udp = new common::BoostUdp(io_service, host_ip_, host_port_, target_ip_, target_port_);
  boost_udp_ = boost_udp;
  boost_udp_->start_sock();
  std::stringstream ss;
  ss  << "\n\tstart udp socket..."
      << "\n\tHost IP:" << host_ip_
      << "\n\tHost port:" << host_port_
      << "\n\tTarget IP:" << target_ip_
      << "\n\tTarget port:" << target_port_
      << "\n\ttopic name:" << topic_name_
      << "\n\tframe ID:" << frame_id_ 
      << "\n";
  ROS_INFO("%s", ss.str().c_str());
  pub_object_ = nh_->advertise<continental_radar_msgs::RadarMsg>(topic_name_, 1000);
  pub_markerarray_ = nh_->advertise<visualization_msgs::MarkerArray>("markerArray", 1000);
  pub_text_ = nh_->advertise<visualization_msgs::MarkerArray>("label", 1000);
}


void ContiRadarDriverRos::run() 
{
  // receive thread for LRR
  boost::function<void()> proc = boost::bind(&drivers::ContiRadarDriverRos::process, this);
  boost::thread main_thread(proc); 
  ROS_INFO("run()....");
}

void ContiRadarDriverRos::process() 
{
    ROS_INFO("start process....");
    drivers::canet::CanetFrame sendout_frame;
    sendout_frame.set_dlc_value(8);
    sendout_frame.set_extended_value(0);
    sendout_frame.set_rtr_value(0);
    sendout_frame.set_id_value(0x200);
    uint8_t output_buf[16];
    // uint8_t* output_buf = const_cast<uint8_t*>(sendout_frame.data8());
    // radar config
    conti_radar_conf_.release_radar_conf();
    conti_radar_conf_.mutable_radar_conf();
    conti_radar_conf_.clear_radar_conf();
    conti_radar_conf_.mutable_radar_conf()->set_max_distance_valid(true);
    conti_radar_conf_.mutable_radar_conf()->set_sensor_id_valid(false);
    conti_radar_conf_.mutable_radar_conf()->set_radar_power_valid(false);
    conti_radar_conf_.mutable_radar_conf()->set_output_type_valid(true);
    conti_radar_conf_.mutable_radar_conf()->set_send_quality_valid(true);
    conti_radar_conf_.mutable_radar_conf()->set_send_ext_info_valid(true);
    conti_radar_conf_.mutable_radar_conf()->set_sort_index_valid(true);
    conti_radar_conf_.mutable_radar_conf()->set_store_in_nvm_valid(true);
    conti_radar_conf_.mutable_radar_conf()->set_ctrl_relay_valid(false);
    conti_radar_conf_.mutable_radar_conf()->set_rcs_threshold_valid(true);

    conti_radar_conf_.mutable_radar_conf()->set_max_distance(100);
    // conti_radar_conf_.mutable_radar_conf()->set_sensor_id(0);
    conti_radar_conf_.mutable_radar_conf()->set_output_type(drivers::conti_radar::OUTPUT_TYPE_OBJECTS);
    // conti_radar_conf_.mutable_radar_conf()->set_radar_power(0);
    // conti_radar_conf_.mutable_radar_conf()->set_ctrl_relay(0);
    conti_radar_conf_.mutable_radar_conf()->set_send_quality(1);
    conti_radar_conf_.mutable_radar_conf()->set_send_ext_info(1);
    conti_radar_conf_.mutable_radar_conf()->set_sort_index(1);
    conti_radar_conf_.mutable_radar_conf()->set_store_in_nvm(1);
    conti_radar_conf_.mutable_radar_conf()->set_rcs_threshold(drivers::conti_radar::RCS_THRESHOLD_HIGH_SENSITIVITY);
    // send config
    
    
    drivers::canbus::ProtocolData<ContiRadar>* protocol_data[9];
    protocol_data[0] = new drivers::conti_radar::RadarState201;
    protocol_data[1] = new drivers::conti_radar::ClusterListStatus600;
    protocol_data[2] = new drivers::conti_radar::ObjectListStatus60A;
    protocol_data[3] = new drivers::conti_radar::ObjectGeneralInfo60B;
    protocol_data[4] = new drivers::conti_radar::ObjectQualityInfo60C;
    protocol_data[5] = new drivers::conti_radar::ObjectExtendedInfo60D;
    protocol_data[6] = new drivers::conti_radar::ClusterGeneralInfo701;
    protocol_data[7] = new drivers::conti_radar::ClusterQualityInfo702;

    conti_radar_.release_radar_state();
    conti_radar_.release_object_list_status();
    conti_radar_.release_cluster_list_status();

    drivers::conti_radar::RadarConfig200 radar_config_200;
    radar_config_200.set_radar_conf(conti_radar_conf_.radar_conf());
    radar_config_200.UpdateData(output_buf);
    ROS_INFO("radar conf: %02X %02X %02X %02X %02X %02X %02X %02X", 
        output_buf[0], output_buf[1], output_buf[2], output_buf[3], 
        output_buf[4], output_buf[5], output_buf[6], output_buf[7] );
    sendout_frame.set_data8_value(output_buf);
    boost_udp_->send_data(sendout_frame.buf(), drivers::canet::CANET_FRAME_LENGTH);
    ROS_INFO("radar configure....");
    ROS_INFO("start receive....");

    while (ros::ok())
    //for(int ii=0; ii<10000; ii++)
    { 
        size_t size = boost_udp_->receive_data(buffer_);
        // size_t size = 0;
//         ROS_INFO("%d bytes received....", size);
        for(int i=0; i<size; i+=drivers::canet::CANET_FRAME_LENGTH)
        {
            drivers::canet::CanetFrame canet_temp(buffer_+i);
            uint32_t id = canet_temp.id(); 
            drivers::canbus::ProtocolData<ContiRadar> *protocol_data_ptr = NULL;
            // protocol_data_ptr
            switch (static_cast<int>(id))
            {
            case drivers::conti_radar::Radar_State :
                if(!conti_radar_.has_radar_state())
                {
                    conti_radar_.mutable_radar_state();
                }
                protocol_data_ptr = protocol_data[0];
//                ROS_INFO("Radar_State received");
                break;
            case drivers::conti_radar::Cluster_List_Status :
                if(conti_radar_.has_cluster_list_status())
                {
                    publish();
                    const_cast<std::vector<drivers::ContiRadarObs>*>(&conti_radar_.contiobs())->clear();
                    conti_radar_.release_object_list_status();
                    conti_radar_.release_cluster_list_status();
                }
                else
                {
                    conti_radar_.mutable_cluster_list_status();
                }
//                ROS_INFO("Cluster_List_Status received");
                protocol_data_ptr = protocol_data[1];
                break;
            case drivers::conti_radar::Object_List_Status :
                if(conti_radar_.has_object_list_status())
                {
                    publish();
                    const_cast<std::vector<drivers::ContiRadarObs>*>(&conti_radar_.contiobs())->clear();
                    conti_radar_.release_object_list_status();
                    conti_radar_.release_cluster_list_status();
                }
                else
                {
                    conti_radar_.mutable_object_list_status();
                }
//                ROS_INFO("Object_List_Status received");
                protocol_data_ptr = protocol_data[2];
                break;
            case drivers::conti_radar::Object_General_Info :
                protocol_data_ptr = protocol_data[3];
                break;
            case drivers::conti_radar::Object_Quality_Info :
                protocol_data_ptr = protocol_data[4];
                break;
            case drivers::conti_radar::Object_Extended_Info :
                protocol_data_ptr = protocol_data[5];
                break;
            case drivers::conti_radar::Cluster_General_Info :
                protocol_data_ptr = protocol_data[6];
                break;
            case drivers::conti_radar::Cluster_Quality_Info :
                protocol_data_ptr = protocol_data[7];
                break;
            default:
               // ROS_WARN("conti_radar - frame ID: %d is out of range.", id);
                break;
            }

	    if (protocol_data_ptr == NULL)
		    continue;

//             ROS_INFO("receive %d", static_cast<int>(id));
            protocol_data_ptr->Parse(canet_temp.data8(),
                canet_temp.dlc(), &conti_radar_ );
//             ROS_INFO("size %d", conti_radar_.contiobs().size());
            if(conti_radar_.has_cluster_list_status())
            {
                int clusters_size = conti_radar_.cluster_list_status().near()
                                    + conti_radar_.cluster_list_status().far();
                if(conti_radar_.contiobs().size() == clusters_size)
                {
//                    ROS_INFO("receive %d clusters.", conti_radar_.contiobs().size());
                    publish();
                    // const_cast<std::vector<drivers::ContiRadarObs>*>(&conti_radar_.contiobs())->clear();
                    conti_radar_.mutable_contiobs()->clear();
                    conti_radar_.release_cluster_list_status();  
                }
                // ROS_INFO("max size %d", clusters_size);
            }

            if(conti_radar_.has_object_list_status())
            {
                int objects_size = conti_radar_.object_list_status().nof_objects();
                if(conti_radar_.contiobs().size() == objects_size)
                {
//                    ROS_INFO("receive %d objects.", conti_radar_.contiobs().size());
                    publish();
                    // const_cast<std::vector<drivers::ContiRadarObs>*>(&conti_radar_.contiobs())->clear();
                    conti_radar_.mutable_contiobs()->clear();
                    conti_radar_.release_object_list_status();  
                }
            }
        }

    }

    for(int i=0; i<8; i++)
    {
        ROS_INFO("delete protocol_data pointer....");
        delete protocol_data[i];
    }
    boost_udp_->close_sock();
}

void ContiRadarDriverRos::publish() 
{
//    ROS_INFO("publish radar objects....");
    continental_radar_msgs::RadarMsg radar_msg;
    continental_radar_msgs::RadarObs radar_obs;
    radar_msg.isvalid = 1;
    radar_msg.header.stamp = ros::Time::now();
    radar_msg.obs.clear();

    visualization_msgs::MarkerArray maker_array;
    visualization_msgs::Marker marker;
    visualization_msgs::MarkerArray array_text;
    maker_array.markers.clear();

    for(auto it : conti_radar_.contiobs())
    {
        // if(it.has_obstacle_class())
        {
            radar_obs.obstacle_id = it.obstacle_id();
            radar_obs.clusterortrack = it.clusterortrack();
            radar_obs.dynprop = it.dynprop();
            radar_obs.lateral_accel = it.lateral_accel();
            radar_obs.lateral_accel_rms = it.lateral_accel_rms();
            radar_obs.lateral_dist = it.lateral_dist();
            radar_obs.lateral_dist_rms = it.lateral_dist_rms();
            radar_obs.lateral_vel = it.lateral_vel();
            radar_obs.lateral_vel_rms = it.lateral_vel_rms();
            radar_obs.length = it.length();
            radar_obs.longitude_accel = it.longitude_accel();
            radar_obs.longitude_accel_rms = it.longitude_accel_rms();
            radar_obs.longitude_dist = it.longitude_dist();
            radar_obs.longitude_dist_rms = it.longitude_dist_rms();
            radar_obs.longitude_vel = it.longitude_vel();
            radar_obs.longitude_vel_rms = it.longitude_vel_rms();
            radar_obs.meas_state = it.meas_state();
            radar_obs.obstacle_class = it.obstacle_class();
            radar_obs.obstacle_id = it.obstacle_id();
            radar_obs.oritation_angle = it.oritation_angle();
            radar_obs.oritation_angle_rms = it.oritation_angle_rms();
            radar_obs.probexist = it.probexist();
            radar_obs.rcs = it.rcs();
            radar_obs.width = it.width();
            // add object to the vector
            radar_msg.obs.push_back(radar_obs);
//            ROS_INFO("id:%d, x:%f y:%f",it.obstacle_id(), radar_obs.longitude_dist, radar_obs.lateral_dist);
        }

        marker.header.frame_id = frame_id_;
        marker.header.stamp = ros::Time::now();
        marker.ns = "continental_radar";

        marker.id =  it.obstacle_id();
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.pose.position.x = radar_obs.longitude_dist;
        marker.pose.position.y = radar_obs.lateral_dist;
        // ROS_INFO("lrrRadarProcData.x:%f",lrrRadarProcData[i].x);
        // ROS_INFO("lrrRadarProcData.y:%f",lrrRadarProcData[i].y);
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        if(marker.pose.position.x > 0.0)
        {
          maker_array.markers.push_back(marker);
          char strTmp[90];
          sprintf(strTmp, "ID:%d, X:%0.2f, Y:%0.2f ", 
                marker.id,
                marker.pose.position.x,
                marker.pose.position.y);
          std::string text_str(strTmp);		
          marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
          marker.pose.position.z = 0.3;
          marker.scale.z = 0.3;
          marker.text = text_str;
          array_text.markers.push_back(marker);
        }
    }   //end of for(auto it : conti_radar_.contiobs())

  //	node_status_pub_ptr_->NODE_ACTIVATE();
  //	node_status_pub_ptr_->CHECK_RATE("RadarMsg", 10, 7, 2, "topic 'RadarMsg' publish rate low.");

    pub_object_.publish(radar_msg);
    pub_markerarray_.publish(maker_array);
    pub_text_.publish(array_text);
    // 
}       //end of ContiRadarDriverRos::publish() 
} // end of namespace drivers

