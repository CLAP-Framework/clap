#include <ros/ros.h>
#include <zzz_perception_detection_lidar_filters/LShapeFilter.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lshape_filter");
    ros::NodeHandle nh, pnh("~");
    zzz::perception::LShapeFilter filter(nh, pnh);
    ros::spin();
}
