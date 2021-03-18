#include "hdmapEngine.h"
#include "ros/ros.h"
#include <string>
#include "std_msgs/String.h"
#include <stdlib.h>

int main (int argc, char** argv) {

  ros::init(argc, argv, "baidu_static_map");
  ros::NodeHandle node;
  HDMapEngine fixed_path_engine(std::string(getenv("ZZZ_ROOT")) + std::string("/zzz/src/map/apollo_hdmap/modules/routing/testdata/routing_tester/routing_test.pb.txt"));
  // publisher and subscriber
  ros::Publisher waypoint_publisher = node.advertise<nav_msgs::Path>(std::string("/carla/ego_vehicle/waypoints"), 10, true);
  ros::Publisher marker_publisher = node.advertise<visualization_msgs::MarkerArray>(std::string("/navigation/markers"), 10);
  ros::Publisher _local_map_publisher = node.advertise<zzz_navigation_msgs::Map>("/zzz/navigation/local_static_map",1);
  ros::Subscriber _pose_subscriber = node.subscribe("/zzz/navigation/ego_pose", 1, &HDMapEngine::egoStateCallback, &fixed_path_engine);
  
  nav_msgs::Path temp = fixed_path_engine.generate_global_trajactory();
  visualization_msgs::MarkerArray msg = fixed_path_engine.generate_all_lanes();
  waypoint_publisher.publish(temp);

  ros::Rate loop_rate(10);
  while (ros::ok()){
    zzz_navigation_msgs::Map::Ptr static_map(new zzz_navigation_msgs::Map());
    if (!fixed_path_engine.update(static_map)){
      _local_map_publisher.publish(static_map);
      ROS_WARN("Map Provider: Publish Local Static Map: lanes_num = %d", int(static_map->lanes.size()));
    }
    //TODO for debug, will remove soon
    // waypoint_publisher.publish(temp);
    marker_publisher.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  return 0;
}
