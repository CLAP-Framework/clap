#define __MODULE_NAME__ "LShapeFilter"
#include <zzz_perception_detection_lidar_filters/LShapeFilter.h>

#include <Eigen/Core>
#include <tf/transform_datatypes.h>
#include <zzz_perception_msgs/DetectionBoxArray.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std;
using namespace pcl;
using namespace Eigen;

namespace zzz { namespace perception {

    LShapeFilter::LShapeFilter(ros::NodeHandle &nh, ros::NodeHandle &pnh)
    {
        string input_topic, output_topic;
        pnh.param("input_topic", input_topic, string("objects_detected"));
        pnh.param("output_topic", output_topic, string("objects_filtered"));
        _input_subscriber = nh.subscribe(input_topic, 1, &LShapeFilter::filter, this);
        _output_publisher = nh.advertise<zzz_perception_msgs::DetectionBoxArray>(output_topic, 1);
    }

    void LShapeFilter::fitLShape(zzz_perception_msgs::DetectionBox &target)
    {
        PointCloud<PointXYZ> cloud;
        fromROSMsg(target.source_cloud, cloud);
        ArrayX2f flat = cloud.getMatrixXfMap().transpose().leftCols(2).array();

        // Find endpoints on both side of the object
        Eigen::Index imin, imax;
        ArrayXf angle = (flat.col(1) / flat.col(0)).atan();
        angle.maxCoeff(&imax); // FIXME(zyxin): fix the case when the angles are crossing pi/2.
        angle.minCoeff(&imin);

        // Calculate distance to the endpoint line (https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line#Line_defined_by_two_points)
        float x1 = flat(imin, 0), y1 = flat(imin, 1), x2 = flat(imax, 0), y2 = flat(imax, 1);
        ArrayXf dist = ((y2-y1)*flat.col(0) - (x2-x1)*flat.col(1) + x2*y1 - x1*y2).abs() / sqrt((y2-y1)*(y2-y1) + (x2-x1)*(x2-x1));
        Eigen::Index iend; dist.maxCoeff(&iend);
        
        // Estimate box shape and orientation
        float x0 = flat(iend, 0), y0 = flat(iend, 1), yaw;
        float dist2min = sqrt((x1-x0)*(x1-x0) + (y1-y0)*(y1-y0));
        float dist2max = sqrt((x2-x0)*(x2-x0) + (y2-y0)*(y2-y0));
        if (dist2max > dist2min)
        {
            yaw = atan2(y2-y0, x2-x0);
            target.bbox.dimension.length_y = dist2max;
            target.bbox.dimension.length_x = dist2min;
            target.bbox.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
        }
        else
        {
            yaw = atan2(y1-y0, x1-x0);
            target.bbox.dimension.length_y = dist2min;
            target.bbox.dimension.length_x = dist2max;
            target.bbox.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
        }
    }

    void LShapeFilter::filter(zzz_perception_msgs::DetectionBoxArrayPtr message)
    {
        for (auto iter = message->detections.begin(); iter != message->detections.end(); iter++)
            fitLShape(*iter);
        _output_publisher.publish(message);
    }

}} // namespace zzz::perception
