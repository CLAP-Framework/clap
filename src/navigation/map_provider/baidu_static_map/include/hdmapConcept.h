#ifndef HDMAP_CONCEPT_H
#define HDMAP_CONCEPT_H
#include <vector>
#include "modules/map/hdmap/hdmap.h"
#include "zzz_navigation_msgs/Lane.h"

class LaneBundle
{
    public:
        LaneBundle(apollo::hdmap::LaneInfoConstPtr& seed);
        void add_path(const std::vector<geometry_msgs::Point>& merged_lane);
        void generate_endpoints(const std::vector<geometry_msgs::Point>& merged_lane);
    public:
        apollo::hdmap::LaneInfoConstPtr lane_seed;
        std::vector<zzz_navigation_msgs::Lane> lanes;
        std::vector<std::vector<float>> central_points;
        float speed_limit;
        std::vector<int> exit_lane_index;
        std::vector<float> traffic_light_pos;
};

#endif