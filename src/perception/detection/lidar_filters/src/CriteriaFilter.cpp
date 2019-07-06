#define __MODULE_NAME__ "CriteriaFilter"
#include <zzz_perception_detection_lidar_filters/CriteriaFilter.h>

using namespace std;

namespace zzz { namespace perception {

    // XXX(zyxin): move common definitions into a base class, including node handle and topic parameters. Could be combined with ros2:rclcpp?
    CriteriaFilter::CriteriaFilter(ros::NodeHandle &nh, ros::NodeHandle &pnh,
        string input_topic, string output_topic)
    {
        _input_subscriber = nh.subscribe(input_topic, 1, &CriteriaFilter::filter, this);
        _output_publisher = nh.advertise<zzz_perception_msgs::DetectionBoxArray>(output_topic, 1);

        pnh.param<float>("max_width", _max_width, 4.f);
        pnh.param<float>("min_width", _min_width, 0.3f);
        pnh.param<float>("max_length", _max_length, 10.f);
        pnh.param<float>("min_length", _min_length, 0.3f);
        pnh.param<float>("max_height", _max_height, 3.f);
        pnh.param<float>("min_height", _min_height, 0.3f);
    }

    bool CriteriaFilter::sizeFilter(zzz_perception_msgs::DetectionBox &target)
    {
        if (target.bbox.dimension.width > _max_width || target.bbox.dimension.width < _min_width)
            return false;
        if (target.bbox.dimension.length > _max_length || target.bbox.dimension.length < _min_length)
            return false;
        if (target.bbox.dimension.height > _max_height || target.bbox.dimension.height < _min_height)
            return false;
        return true;
    }

    void CriteriaFilter::filter(zzz_perception_msgs::DetectionBoxArrayPtr message)
    {
        zzz_perception_msgs::DetectionBoxArray filtered;
        for (auto iter = message->detections.begin(); iter != message->detections.end(); iter++)
            if (sizeFilter(*iter))
                filtered.detections.push_back(*iter);

        filtered.header = message->header;
        _output_publisher.publish(filtered);
    }

}} // namespace zzz::perception

