#include "lidar_track_ros.h"
#include <ros/console.h>
#include <iostream>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "simple_ukf_track");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Error);
    // ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Fatal);
    LidarTrackRos app;
    app.Run();
    ros::spin();

    return 0; 
}