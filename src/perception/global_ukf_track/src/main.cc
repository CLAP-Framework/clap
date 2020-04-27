#include "lidar_track_ros.h"
// #include "lidar_track.h"
#include <iostream>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "simple_ukf_track");

    LidarTrackRos app;
    app.Run();
    ros::spin();

    return 0; 
}