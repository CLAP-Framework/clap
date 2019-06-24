#include <ros/ros.h>
#include <zzz_visualization_rviz_detection_visualizer/BBoxVisualizer.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bbox_visualizer");
    ros::NodeHandle nh, pnh("~");
    zzz::visualization::rviz::BBoxVisualizer visualizer(nh, pnh);
    ros::spin();
}
