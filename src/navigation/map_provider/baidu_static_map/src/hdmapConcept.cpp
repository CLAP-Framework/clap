#include "hdmapConcept.h"

LaneBundle::LaneBundle(apollo::hdmap::LaneInfoConstPtr& seed)
{
    lane_seed = seed;
}

void LaneBundle::add_path(const std::vector<geometry_msgs::Point>& merged_lane)
{
    zzz_navigation_msgs::Lane temp_lane = zzz_navigation_msgs::Lane();
    for (int i=0; i < merged_lane.size(); i++){
        zzz_navigation_msgs::LanePoint temp_point = zzz_navigation_msgs::LanePoint();
        temp_point.position.x = merged_lane[i].x;
        temp_point.position.y = merged_lane[i].y;
        temp_lane.central_path_points.emplace_back(temp_point);
    }
    temp_lane.speed_limit = this->speed_limit;
    for (auto& pos : this->traffic_light_pos){
        temp_lane.traffic_light_pos.emplace_back(pos);
    }
    this->lanes.emplace_back(temp_lane);
}

void LaneBundle::generate_endpoints(const std::vector<geometry_msgs::Point>& merged_lane)
{
    for (auto& point : merged_lane){
        this->central_points.push_back({float(point.x), float(point.y)});
    }
}