#ifndef ZZZ_PERCEPTION_DETECTION_LIDAR_FILTERS_LSHAPEFILTER_H
#define ZZZ_PERCEPTION_DETECTION_LIDAR_FILTERS_LSHAPEFILTER_H

#include <ros/ros.h>
#include <zzz_perception_msgs/DetectionBoxArray.h>

namespace zzz
{
    namespace perception
    {
        class LShapeFilter
        {
        public:
            LShapeFilter(ros::NodeHandle &node_handle, ros::NodeHandle &private_handle);

        protected:
            ros::Subscriber _input_subscriber;
            ros::Publisher _output_publisher;

            void fitLShape(zzz_perception_msgs::DetectionBox &target);

        private:
            void filter(zzz_perception_msgs::DetectionBoxArrayPtr message);
        };
    } // namespace perception
} // namespace zzz

#endif // ZZZ_PERCEPTION_DETECTION_LIDAR_FILTERS_LSHAPEFILTER_H
