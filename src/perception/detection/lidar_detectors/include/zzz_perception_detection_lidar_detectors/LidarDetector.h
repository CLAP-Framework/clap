#ifndef ZZZ_PERCEPTION_DETECTION_LIDAR_DETECTORS_LIDARDETECTOR_H
#define ZZZ_PERCEPTION_DETECTION_LIDAR_DETECTORS_LIDARDETECTOR_H

#include <string>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

namespace zzz
{
    namespace perception
    {
        class LidarDetector
        {
        public:
            LidarDetector(ros::NodeHandle &node_handle, ros::NodeHandle &private_handle);
            virtual void detect(sensor_msgs::PointCloud2ConstPtr input)=0;

        protected:
            ros::Subscriber _input_subscriber;
            ros::Publisher _output_publisher;

            std::string _frame_id; // store for debug use
            ros::Time _timestamp; // store for debug use
        };
    } // namespace perception
} // namespace zzz

#endif // ZZZ_PERCEPTION_DETECTION_LIDAR_DETECTORS_LIDARDETECTOR_H
