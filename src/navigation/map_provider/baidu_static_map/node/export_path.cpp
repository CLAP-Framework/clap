#include "hdmapEngine.h"
#include "ros/ros.h"
#include <string>
#include <iostream>
#include <fstream>
#include "std_msgs/String.h"
#include <stdlib.h>
using namespace std;

int main (int argc, char** argv) {

  ros::init(argc, argv, "baidu_static_map");
  ros::NodeHandle node;
  HDMapEngine fixed_path_engine(std::string(getenv("ZZZ_ROOT")) + std::string("/zzz/src/map/apollo_hdmap/modules/routing/testdata/routing_tester/routing_test.pb.txt"));
  // publisher and subscriber
  
  nav_msgs::Path temp_global_path = fixed_path_engine.generate_global_trajactory();
  ofstream global_path_file;
  global_path_file.open("global_path", ios::out);
  for (auto& point : temp_global_path.poses){
    global_path_file << point.pose.position.x << "," << point.pose.position.y << endl;
  }
  global_path_file.close();
  fixed_path_engine.dump_lanes_info();

  return 0;
}
