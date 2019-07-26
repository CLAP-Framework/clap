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
            CriteriaFilter(ros::NodeHandle &node_handle, ros::NodeHandle &private_handle);

        protected:
            ros::Subscriber _input_subscriber;
            ros::Publisher _output_publisher;

            bool sizeFilter(zzz_perception_msgs::DetectionBox &target);

        private:
            float _max_length_x, _min_length_x, _max_length_y, _min_length_y, _max_length_z, _min_length_z;

            void filter(zzz_perception_msgs::DetectionBoxArrayPtr message);
        };
    } // namespace perception
} // namespace zzz

#endif // ZZZ_PERCEPTION_DETECTION_LIDAR_FILTERS_CRITERIAFILTER_H
