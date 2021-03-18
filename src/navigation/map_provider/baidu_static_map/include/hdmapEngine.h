#include "ros/ros.h"
#include <ros/console.h>
#include "nav_msgs/Path.h"
#include "hdmapConcept.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"
#include "zzz_navigation_msgs/Map.h"
#include "zzz_driver_msgs/RigidBodyStateStamped.h"
// hdmap includes
#include "modules/routing/routing.h"
#include "cyber/common/file.h"
#include "modules/routing/proto/routing.pb.h"

#include <visualization_msgs/MarkerArray.h>

#include <cmath>
#include <map>
#include <mutex>
#include <vector>
#include <memory>
#include <iomanip>
#include <iostream>
#include <string>
#include <yaml-cpp/yaml.h>
#include <fstream>


class HDMapEngine{
    public:
        HDMapEngine();
        HDMapEngine(std::string waypoint_file_path);
        nav_msgs::Path generate_global_trajactory();
        visualization_msgs::MarkerArray generate_all_lanes();
        void egoStateCallback(const zzz_driver_msgs::RigidBodyStateStamped::ConstPtr& msg);
        int update(const zzz_navigation_msgs::Map::Ptr& static_map);
        int exportCurrentPosition(float* start_x, float* start_y);
        void generate_routing_request(float s_x, float s_y, float e_x, float e_y);
        void dump_lanes_info();

    private:
        void print_lane_info(const apollo::hdmap::LaneInfoConstPtr & lane, std::string title);
        std::vector<apollo::hdmap::LaneInfoConstPtr> get_all_lanes_after_junction();
        std::vector<geometry_msgs::Point> merge_lanes(const std::vector<apollo::hdmap::LaneInfoConstPtr>& lane_pieces);
        std::vector<std::vector<geometry_msgs::Point>> get_parallel_lanes_until_junction(const apollo::hdmap::LaneInfoConstPtr &start_lane,
                                                                                        LaneBundle* laneBundle);
        void get_next_sublane_until_sudo_junction(const apollo::hdmap::LaneInfoConstPtr &start_lane, std::vector<apollo::hdmap::LaneInfoConstPtr>* lanes);
        void get_lane_info(const apollo::routing::LaneSegment &temp_lane_seg);
        bool is_lane_successor_at_intersection(const apollo::hdmap::LaneInfoConstPtr &lane);
        bool is_lane_divergent(const apollo::hdmap::LaneInfoConstPtr &lane);
        double calculate_distance(const apollo::common::PointENU& start_point, const apollo::common::PointENU& end_point);
        int get_partial_lane(const apollo::routing::LaneSegment &temp_lane_seg, nav_msgs::Path* global_trajactory);
        void get_lane(const apollo::routing::LaneSegment &temp_lane_seg, nav_msgs::Path* global_trajactory);
        void get_half_lane(const apollo::routing::LaneSegment &temp_lane_seg, nav_msgs::Path* global_trajactory, int number);
        void get_left_lanes(const apollo::hdmap::LaneInfoConstPtr &start_lane, std::vector<apollo::hdmap::LaneInfoConstPtr>* parallel_lanes);
        void get_right_lanes(const apollo::hdmap::LaneInfoConstPtr &start_lane, std::vector<apollo::hdmap::LaneInfoConstPtr>* parallel_lanes);
        void generate_distances_array();
        int generate_cloest_lane_index();
        std::vector<std::string> split(const std::string& srcstr, const std::string& delimeter);
        std::vector<std::pair<float,float>> linear_interpolate(float start_x, float start_y, float end_x, float end_y, float resolution=1);
        std::vector<apollo::common::PointENU> linear_interpolate(const apollo::common::PointENU& start_point, const apollo::common::PointENU& end_point);
    private:
        apollo::hdmap::HDMap map;
        apollo::routing::Routing routing_;
        apollo::routing::LaneSegment last_lane_segment;
        apollo::routing::RoutingRequest routing_request;
        std::shared_ptr<apollo::routing::RoutingRequest> routing_request_ptr;
        apollo::routing::RoutingResponse routing_response;
        std::vector<apollo::hdmap::LaneInfoConstPtr> complete_lane;
        std::vector<LaneBundle> lane_output_info;
        std::vector<std::vector<std::pair<float, float>>> lanes_distance_info;
        zzz_driver_msgs::RigidBodyStateStamped::ConstPtr _ego_vehicle_state_buffer;
        std::mutex _ego_vehicle_state_lock;

};