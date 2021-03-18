#include "hdmapEngine.h"
#include "routeSelectingWindow.h"
#include "ros/ros.h"
#include <string>
#include "std_msgs/String.h"

int main (int argc, char** argv) {

  ros::init(argc, argv, "baidu_static_map");
  ros::NodeHandle node;
  HDMapEngine user_path_engine;
  // publisher and subscriber
  ros::Publisher waypoint_publisher = node.advertise<nav_msgs::Path>(std::string("/carla/ego_vehicle/waypoints"), 10, true);
  ros::Publisher marker_publisher = node.advertise<visualization_msgs::MarkerArray>(std::string("/navigation/markers"), 10);
  ros::Publisher _local_map_publisher = node.advertise<zzz_navigation_msgs::Map>("/zzz/navigation/local_static_map",1);
  ros::Subscriber _pose_subscriber = node.subscribe(std::string("/zzz/navigation/ego_pose"), 1, &HDMapEngine::egoStateCallback, &user_path_engine);
  
  float destination_x, destination_y;
  QApplication get_endpoint(argc, argv);
  RoutingWindow routing_window(&destination_x, &destination_y);
  routing_window.show();
  get_endpoint.exec();

  float start_x, start_y;
  ros::Rate pose_rate(10);
  while (ros::ok()){
    ros::spinOnce();
    ROS_WARN("Map Provider: Trying to get ego pose for routing");
    if (!user_path_engine.exportCurrentPosition(&start_x, &start_y)){
      break;
    }
    pose_rate.sleep();
  }
  user_path_engine.generate_routing_request(start_x, start_y, destination_x, destination_y);

  nav_msgs::Path temp = user_path_engine.generate_global_trajactory();
  visualization_msgs::MarkerArray msg = user_path_engine.generate_all_lanes();
  waypoint_publisher.publish(temp);

  ros::Rate loop_rate(10);
  while (ros::ok()){
    zzz_navigation_msgs::Map::Ptr static_map(new zzz_navigation_msgs::Map());
    if (!user_path_engine.update(static_map)){
      _local_map_publisher.publish(static_map);
      ROS_INFO("Map Provider: Publish Local Static Map: lanes_num = %d", int(static_map->lanes.size()));
    }
    //TODO for debug, will remove soon
    // waypoint_publisher.publish(temp);
    marker_publisher.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  return 0;
}
