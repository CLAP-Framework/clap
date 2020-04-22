#include "zzz_perception_detection_lidar_detectors/LidarDetector.h"
#include <zzz_perception_msgs/DetectionBoxArray.h>

using namespace std;

namespace zzz { namespace perception {
    LidarDetector::LidarDetector(ros::NodeHandle &node_handle, ros::NodeHandle &private_handle)
    {
        string input_topic, output_topic;
        private_handle.param("input_topic", input_topic, string("points_raw"));
        private_handle.param("output_topic", output_topic, string("objects_detected"));

        _input_subscriber = node_handle.subscribe(input_topic, 1, &LidarDetector::detect, this);
        _output_publisher = node_handle.advertise<zzz_perception_msgs::DetectionBoxArray>(output_topic, 1);
    }
}} // namespace zzz::perception
