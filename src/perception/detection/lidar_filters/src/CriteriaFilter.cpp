#define __MODULE_NAME__ "CriteriaFilter"
#include <zzz_perception_detection_lidar_filters/CriteriaFilter.h>

using namespace std;

namespace zzz { namespace perception {

    // XXX(zyxin): move common definitions into a base class, including node handle and topic parameters. Could be combined with ros2:rclcpp?
    CriteriaFilter::CriteriaFilter(ros::NodeHandle &nh, ros::NodeHandle &pnh)
    {
        string input_topic, output_topic;
        pnh.param("input_topic", input_topic, string("objects_detected"));
        pnh.param("output_topic", output_topic, string("objects_filtered"));
        _input_subscriber = nh.subscribe(input_topic, 1, &CriteriaFilter::filter, this);
        _output_publisher = nh.advertise<zzz_perception_msgs::DetectionBoxArray>(output_topic, 1);

        pnh.param<float>("max_length_x", _max_length_x, 4.f); // width
        pnh.param<float>("min_length_x", _min_length_x, 0.3f);
        pnh.param<float>("max_length_y", _max_length_y, 10.f); // length
        pnh.param<float>("min_length_y", _min_length_y, 0.3f);
        pnh.param<float>("max_length_z", _max_length_z, 3.f); // height
        pnh.param<float>("min_length_z", _min_length_z, 0.3f);
    }

    bool CriteriaFilter::sizeFilter(zzz_perception_msgs::DetectionBox &target)
    {
        if (target.bbox.dimension.length_x > _max_length_x || target.bbox.dimension.length_x < _min_length_x)
            return false;
        if (target.bbox.dimension.length_y > _max_length_y || target.bbox.dimension.length_y < _min_length_y)
            return false;
        if (target.bbox.dimension.length_z > _max_length_z || target.bbox.dimension.length_z < _min_length_z)
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

