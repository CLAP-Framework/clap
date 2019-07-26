#ifndef ZZZ_PERCEPTION_DETECTION_LIDAR_DETECTORS_EUCLIDEANCLUSTERDETECTOR_H
#define ZZZ_PERCEPTION_DETECTION_LIDAR_DETECTORS_EUCLIDEANCLUSTERDETECTOR_H

#include "zzz_perception_detection_lidar_detectors/LidarDetector.h"

#include <pcl/PointIndices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "zzz_perception_msgs/DetectionBoxArray.h"

namespace zzz
{
    namespace perception
    {
        class EuclideanClusterDetector : LidarDetector
        {
        private: // configurations
            bool _downsampling_enable;
            float _downsampling_leaf_size;

            bool _crop_enable;
            float _crop_min_z;
            float _crop_max_z;

            bool _plane_removal_enable;
            float _plane_removal_ransac_thres;
            float _plane_removal_count_thres;

            float _cluster_dist_thres;
            int _cluster_count_min;
            int _cluster_count_max;

        public:
            EuclideanClusterDetector(ros::NodeHandle &node_handle, ros::NodeHandle &private_handle);
            virtual void detect(sensor_msgs::PointCloud2ConstPtr input);
        
        protected:
            #ifndef NDEBUG
            ros::Publisher _cluster_publisher;
            ros::Publisher _plane_publisher;
            #endif

            pcl::PointIndices::Ptr planeRemove(
                pcl::PointCloud<pcl::PointXYZ>::ConstPtr input,
                pcl::PointIndices::ConstPtr indices);
            void segment(pcl::PointCloud<pcl::PointXYZ>::ConstPtr input,
                         pcl::PointIndices::ConstPtr indices,
                         std::vector<pcl::PointIndices> &clusters);
            void estimate(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cluster_input,
                          zzz_perception_msgs::BoundingBox &detection);
        };
    } // namespace perception
} // namespace zzz

#endif // ZZZ_PERCEPTION_DETECTION_LIDAR_DETECTORS_EUCLIDEANCLUSTERDETECTOR
