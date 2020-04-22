#ifndef ZZZ_PERCEPTION_DETECTION_LIDAR_DETECTORS_EUCLIDEANCLUSTER2DDETECTOR_H
#define ZZZ_PERCEPTION_DETECTION_LIDAR_DETECTORS_EUCLIDEANCLUSTER2DDETECTOR_H

#include "zzz_perception_detection_lidar_detectors/LidarDetector.h"

namespace zzz
{
    namespace perception
    {
        class EuclideanCluster2dDetector : LidarDetector
        {
        public:
            EuclideanCluster2dDetector(ros::NodeHandle &node_handle, ros::NodeHandle &private_handle,
                std::string input_topic="points_raw", std::string output_topic="objects_detected");
            virtual void detect(sensor_msgs::PointCloud2ConstPtr input);
        
        private:
        };
    } // namespace perception
} // namespace zzz

#endif // ZZZ_PERCEPTION_DETECTION_LIDAR_DETECTORS_EUCLIDEANCLUSTER2DDETECTOR_H
