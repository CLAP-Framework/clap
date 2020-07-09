
#include <cstdlib>
#include <string>
#include <sstream>
#include <cstdint>
#include "srr_radar/srr_radar_driver_ros.h"

#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include "visualization_msgs/MarkerArray.h"
#include "srr_radar/common/boost_udp.h"
#include "srr_radar/proto/srr_radar.h"
#include "srr_radar/proto/srr_radar_conf.h"
#include "srr_radar/frame/canet/canet.h"
#include "srr_radar/protocol/const_vars.h"
#include "srr_radar/protocol/radar_config_200.h"
#include "srr_radar/protocol/radar_state_60a.h"
#include "srr_radar/protocol/track_list_status_60b.h"
#include "srr_radar/protocol/track_1_60c.h"
#include "srr_radar/protocol/track_2_60d.h"
#include "srr_radar/protocol/cluster_list_status_70b.h"
#include "srr_radar/protocol/cluster_1_70c.h"
#include "srr_radar/common/param.h"
#include "srr_radar_msgs/RadarMsg.h"        //TODO where is this .h file???

namespace drivers {

void SrrRadarDriverRos::init(const ros::NodeHandle* parent)             //TODO: change IP and port
{
  //
  // ros::NodeHandle nh2("~");   
  nh_ = const_cast<ros::NodeHandle*>(parent);

  host_ip_ = getParam < std::string >("srr_radar/Host_Addr", "192.168.110.105");
  host_port_ = getParam < int > ("srr_radar/Host_Port", 4005);
  target_ip_ = getParam < std::string >("srr_radar/Target_Addr", "192.168.110.103");
  target_port_ = getParam < int >("srr_radar/Target_Port", 4005);
  topic_name_ = getParam < std::string>("srr_radar/Topic_Name", "SrrRadarMsg");
  frame_id_ = getParam < std::string> ("srr_radar/Frame_ID", "srr_radar");

 // node_status_pub_ptr_ = std::make_shared<health_checker::NodeStatusPublisher>(*nh_);
 // node_status_pub_ptr_->ENABLE();

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
  pub_object_ = nh_->advertise<srr_radar_msgs::RadarMsg>(topic_name_, 1000);
  pub_markerarray_ = nh_->advertise<visualization_msgs::MarkerArray>("markerArray", 1000);
  pub_text_ = nh_->advertise<visualization_msgs::MarkerArray>("label", 1000);
}

void SrrRadarDriverRos::run() 
{
  // receive thread for LRR
  boost::function<void()> proc = boost::bind(&drivers::SrrRadarDriverRos::process, this);
  boost::thread main_thread(proc); 
  ROS_INFO("run()....");
}

void SrrRadarDriverRos::process() 
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
    srr_radar_conf_.release_radar_conf();
    srr_radar_conf_.mutable_radar_conf();
    srr_radar_conf_.clear_radar_conf();

    srr_radar_conf_.mutable_radar_conf()->set_sensor_id_valid(false);
    srr_radar_conf_.mutable_radar_conf()->set_output_type_valid(true);
    // srr_radar_conf_.mutable_radar_conf()->set_sensor_id(0);
    //Use track for SRR. Cluster not used at this moment.
    srr_radar_conf_.mutable_radar_conf()->set_output_type(drivers::srr_radar::OUTPUT_TYPE_TRACKS);
    // send confi    
    
    drivers::canbus::ProtocolData<SrrRadar>* protocol_data[7];
    protocol_data[0] = new drivers::srr_radar::RadarState60A;
    protocol_data[1] = new drivers::srr_radar::TrackListStatus60B;
    protocol_data[2] = new drivers::srr_radar::Track1_60C;
    protocol_data[3] = new drivers::srr_radar::Track2_60D;
    protocol_data[4] = new drivers::srr_radar::ClusterListStatus70B;
    protocol_data[5] = new drivers::srr_radar::Cluster1_70C;

    srr_radar_.release_radar_state();
    srr_radar_.release_track_list_status();
    srr_radar_.release_cluster_list_status();

    drivers::srr_radar::RadarConfig200 radar_config_200;
    radar_config_200.set_radar_conf(srr_radar_conf_.radar_conf());
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
        for(int i=0; i<size; i+=drivers::canet::CANET_FRAME_LENGTH)     //13
        {
            drivers::canet::CanetFrame canet_temp(buffer_+i);
            uint32_t id = canet_temp.id(); 
            drivers::canbus::ProtocolData<SrrRadar> *protocol_data_ptr = NULL;
            // protocol_data_ptr
            switch (static_cast<int>(id))
            {
            case drivers::srr_radar::Radar_State :
                if(!srr_radar_.has_radar_state())
                {
                    srr_radar_.mutable_radar_state();
                }
                protocol_data_ptr = protocol_data[0];
//                ROS_INFO("Radar_State received");
                break;

            case drivers::srr_radar::Track_List_Status :
                if(srr_radar_.has_track_list_status())
                {
                    publish();
                    const_cast<std::vector<drivers::SrrRadarObs>*>(&srr_radar_.srrobs())->clear();
                    srr_radar_.release_track_list_status();
                    srr_radar_.release_cluster_list_status();
                }
                else
                {
                    srr_radar_.mutable_track_list_status();
                }
//                ROS_INFO("Track_List_Status received");
                protocol_data_ptr = protocol_data[1];
                break;
            case drivers::srr_radar::Track_1 :
                protocol_data_ptr = protocol_data[2];
                break;
            case drivers::srr_radar::Track_2 :
                protocol_data_ptr = protocol_data[3];
                break;

            case drivers::srr_radar::Cluster_List_Status :
                if(srr_radar_.has_cluster_list_status())
                {
                    publish();
                    const_cast<std::vector<drivers::SrrRadarObs>*>(&srr_radar_.srrobs())->clear();
                    srr_radar_.release_track_list_status();
                    srr_radar_.release_cluster_list_status();
                }
                else
                {
                    srr_radar_.mutable_cluster_list_status();
                }
//                ROS_INFO("Cluster_List_Status received");
                protocol_data_ptr = protocol_data[4];
                break;
            case drivers::srr_radar::Cluster_1 :
                protocol_data_ptr = protocol_data[5];
                break;
            default:
               // ROS_INFO("srr_radar - frame ID: %d is out of range.", id);
                break;
            }

	    if (protocol_data_ptr == NULL)
		    continue;

//             ROS_INFO("receive %d", static_cast<int>(id));
            protocol_data_ptr->Parse(canet_temp.data8(),
                canet_temp.dlc(), &srr_radar_ );
//             ROS_INFO("size %d", srr_radar_.srrobs().size());
            if(srr_radar_.has_cluster_list_status())
            {
                int clusters_size = srr_radar_.cluster_list_status().num_of_cluster();
                if(srr_radar_.srrobs().size() == clusters_size)     //already get same number clusters with list content
                {
//                    ROS_INFO("receive %d clusters.", srr_radar_.srrobs().size());
                    publish();
                    // const_cast<std::vector<drivers::SrrRadarObs>*>(&srr_radar_.srrobs())->clear();
                    srr_radar_.mutable_srrobs()->clear();
                    srr_radar_.release_cluster_list_status();  
                }
                // ROS_INFO("max size %d", clusters_size);
            }

            if(srr_radar_.has_track_list_status())
            {
                int tracks_size = srr_radar_.track_list_status().num_of_tracks();
                if(srr_radar_.srrobs().size() == tracks_size)     //already get same number clusters with list content
                {
//                    ROS_INFO("receive %d tracks.", srr_radar_.srrobs().size());
                    publish();
                    // const_cast<std::vector<drivers::SrrRadarObs>*>(&srr_radar_.srrobs())->clear();
                    srr_radar_.mutable_srrobs()->clear();
                    srr_radar_.release_track_list_status();  
                }
            }
        }   //end of  for(int i=0; i<size; i+=drivers::canet::CANET_FRAME_LENGTH)

    }   //end of while (ros::ok())

    for(int i=0; i<6; i++)
    {
        ROS_INFO("delete protocol_data pointer....");
        delete protocol_data[i];
    }
    boost_udp_->close_sock();
}

void SrrRadarDriverRos::publish() 
{
//    ROS_INFO("publish radar tracks....");
    srr_radar_msgs::RadarMsg radar_msg;
    srr_radar_msgs::RadarObs radar_obs;
    radar_msg.isvalid = 1;
    radar_msg.header.stamp = ros::Time::now();
    radar_msg.obs.clear();

    visualization_msgs::MarkerArray maker_array;
    visualization_msgs::Marker marker;
    visualization_msgs::MarkerArray array_text;
    maker_array.markers.clear();

    for(auto it : srr_radar_.srrobs())
    {
        // check the objects data intergrity        
        // if(intergrity)
        // if(it.has_obstacle_class())
        {
            radar_obs.obstacle_id = it.obstacle_id();
            radar_obs.clusterortrack = it.clusterortrack();
            radar_obs.obstacle_class = 0;
            //track1
            radar_obs.longitude_displ = it.track_longitude_displ();
            radar_obs.lateral_displ = it.track_lateral_displ();
            radar_obs.length = 0;
            radar_obs.width = 0;
            radar_obs.height = 0;
            radar_obs.longitude_vel = it.track_longitude_vel();
            radar_obs.lateral_vel = it.track_lateral_vel();
            radar_obs.longitude_acc = 0;
            radar_obs.lateral_acc = 0;
            radar_obs.obstacle_class_quality = 0;
            //track2
            radar_obs.rcs_value = it.rcs_value();
            radar_obs.track_lifetime = it.track_lifetime();  
             //cluster
            radar_obs.range = it.range();
            radar_obs.azimuth = it.azimuth();
            radar_obs.vrel = it.vrel();
            // add object to the vector
            radar_msg.obs.push_back(radar_obs);
//            ROS_INFO("id:%d, x:%f y:%f",it.obstacle_id(), radar_obs.longitude_dist, radar_obs.lateral_dist);
        }

        marker.header.frame_id = frame_id_;
        marker.header.stamp = ros::Time::now();
        marker.ns = "srr_radar";

        marker.id =  it.obstacle_id();
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.pose.position.x = radar_obs.longitude_displ;
        marker.pose.position.y = radar_obs.lateral_vel;
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
    }   //end of for(auto it : srr_radar_.srrobs())

 //   node_status_pub_ptr_->NODE_ACTIVATE();
 //   node_status_pub_ptr_->CHECK_RATE("SrrRadarMsg", 24, 15, 3, "topic 'SrrRadarMsg' publish rate low.");

    pub_object_.publish(radar_msg);
    pub_markerarray_.publish(maker_array);
    pub_text_.publish(array_text);
    // 
}       //end of SrrRadarDriverRos::publish() 
} // end of namespace drivers

