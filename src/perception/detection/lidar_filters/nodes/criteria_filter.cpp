#include <ros/ros.h>
#include <zzz_perception_detection_lidar_filters/CriteriaFilter.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "criteria_filter");
    ros::NodeHandle nh, pnh("~");
    zzz::perception::CriteriaFilter filter(nh, pnh);
    ros::spin();
}
