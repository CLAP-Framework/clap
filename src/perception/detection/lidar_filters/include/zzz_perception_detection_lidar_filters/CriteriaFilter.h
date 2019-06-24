#ifndef ZZZ_PERCEPTION_DETECTION_LIDAR_FILTERS_CRITERIAFILTER_H
#define ZZZ_PERCEPTION_DETECTION_LIDAR_FILTERS_CRITERIAFILTER_H

#include <ros/ros.h>
#include <zzz_perception_msgs/DetectionBoxArray.h>

namespace zzz
{
    namespace perception
    {
        class CriteriaFilter
        {
        public:
            CriteriaFilter(ros::NodeHandle &node_handle, ros::NodeHandle &private_handle,
                std::string input_topic="objects_detected", std::string output_topic="objects_filtered");

        protected:
            ros::Subscriber _input_subscriber;
            ros::Publisher _output_publisher;

            bool sizeFilter(zzz_perception_msgs::DetectionBox &target);

        private:
            float _max_width, _min_width, _max_length, _min_length, _max_height, _min_height;

            void filter(zzz_perception_msgs::DetectionBoxArrayPtr message);
        };
    } // namespace perception
} // namespace zzz

#endif // ZZZ_PERCEPTION_DETECTION_LIDAR_FILTERS_CRITERIAFILTER_H
