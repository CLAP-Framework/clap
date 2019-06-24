#include <ros/ros.h>
#include <zzz_perception_detection_lidar_detectors/EuclideanClusterDetector.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "euclidean_cluster_detector");
    ros::NodeHandle nh, pnh("~");
    zzz::perception::EuclideanClusterDetector detector(nh, pnh);
    ros::spin();
}
