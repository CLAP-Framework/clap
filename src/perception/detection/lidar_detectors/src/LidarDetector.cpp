#include "zzz_perception_detection_lidar_detectors/LidarDetector.h"
#include <zzz_perception_msgs/DetectionBoxArray.h>

namespace zzz { namespace perception {
    LidarDetector::LidarDetector(ros::NodeHandle &node_handle, ros::NodeHandle &private_handle,
        std::string input_topic, std::string output_topic)
    {
        _input_subscriber = node_handle.subscribe(input_topic, 1, &LidarDetector::detect, this);
        _output_publisher = node_handle.advertise<zzz_perception_msgs::DetectionBoxArray>(output_topic, 1);
    }
}} // namespace zzz::perception
