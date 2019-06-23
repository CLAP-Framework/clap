#define __MODULE_NAME__ "BBoxVisualizer"
#include "zzz_visualization_rviz_detection_visualizer/BBoxVisualizer.h"

using namespace std;

namespace zzz { namespace visualization { namespace rviz {

    inline float checkColor(float value)
    {
        if (value > 255.)
            return 1.f;
        else if (value < 0)
            return 0.f;
        else
            return value / 255.f;
    }
    inline float checkAlpha(float value)
    {
        if (value > 1.)
            return 1.f;
        else if (value < 0.1)
            return 0.1f;
        else
            return value;
    }

    // Colors should be in order r, g, b, a
    std_msgs::ColorRGBA parseColor(const std::vector<float> &in_color)
    {
        std_msgs::ColorRGBA color;
        if (in_color.size() == 4)
        {
            color.r = checkColor(in_color[0]);
            color.g = checkColor(in_color[1]);
            color.b = checkColor(in_color[2]);
            color.a = checkAlpha(in_color[3]);
        }
        else
        {
            ROS_ERROR("[%s] Cannot resolve color value. Check configuration please!", __MODULE_NAME__);
        }
        return color;
    }

    BBoxVisualizer::BBoxVisualizer(ros::NodeHandle &nh, ros::NodeHandle &pnh,
        string input_topic, string output_topic, string marker_namespace)
    {
        // Resolve visualization type and subscribe
        string type_name;
        pnh.param<string>("visualize_type", type_name, "DetectionBoxArray");
        if (type_name == "DetectionBoxArray")
        {
            _vis_type = BBoxVisualizerType::DetectionBoxArray;
            _input_subscriber = nh.subscribe(input_topic, 1, &BBoxVisualizer::visualizeDetection, this);
        }
        else if (type_name == "TrackingBoxArray")
        {
            _vis_type = BBoxVisualizerType::TrackingBoxArray;
            _input_subscriber = nh.subscribe(input_topic, 1, &BBoxVisualizer::visualizeTracking, this);
        }
        else
        {
            ROS_ERROR("[%s] Cannot resolve visualization type. Check configuration please!", __MODULE_NAME__);
        }
        _output_publisher = pnh.advertise<visualization_msgs::MarkerArray>(output_topic, 1);
        _marker_namespace = marker_namespace;

        // General parameters
        pnh.param<float>("marker_lifetime", _marker_lifetime, 0.2f);

        std::vector<float> color;
        pnh.param<std::vector<float>>("label_color", color, {255.,255.,255.,1.0});
        _label_color = parseColor(color);
        pnh.param<std::vector<float>>("box_color", color, {51.,128.,204.,0.8});
        _box_color = parseColor(color);
        pnh.param<std::vector<float>>("centroid_color", color, {77.,121.,255.,0.8});
        _centroid_color = parseColor(color);

        pnh.param<float>("centroid_scale", _centroid_scale, 0.5f);
        pnh.param<float>("label_scale", _label_scale, 0.5f);
        pnh.param<float>("label_height", _label_scale, 1.0f);
        pnh.param<float>("box_max_size", _box_max_size, 10);
    }

    void BBoxVisualizer::addLabels(const zzz_perception_msgs::DetectionBoxArray &in_objects, visualization_msgs::MarkerArray &out_objects)
    {
        for (auto const &object: in_objects.detections)
        {
            visualization_msgs::Marker label_marker;
            label_marker.lifetime = ros::Duration(_marker_lifetime);

            // Message headers
            label_marker.header = in_objects.header;
            label_marker.action = visualization_msgs::Marker::ADD;
            label_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            label_marker.ns = _marker_namespace + "/label_markers";
            label_marker.id = _marker_id++;

            // Marker properties
            label_marker.scale.x = _label_scale;
            label_marker.scale.y = _label_scale;
            label_marker.scale.z = _label_scale;
            label_marker.color = _label_color;

            if(object.classes.size() > 0) // Use classification name if presented
                label_marker.text = "[" + to_string(object.classes.front().classid) + "]";

            float x = object.bbox.pose.pose.position.x;
            float y = object.bbox.pose.pose.position.y;
            std::stringstream distance_stream;
            distance_stream << std::fixed << std::setprecision(1) << sqrt(x*x + y*y) << " m";
            label_marker.text += distance_stream.str();

            label_marker.pose.position.x = x;
            label_marker.pose.position.y = y;
            label_marker.pose.position.z = _label_height;
            
            out_objects.markers.push_back(label_marker);
        }
    }

    void BBoxVisualizer::addBoxes(const zzz_perception_msgs::DetectionBoxArray &in_objects, visualization_msgs::MarkerArray &out_objects)
    {
        for (auto const &object: in_objects.detections)
        {
            visualization_msgs::Marker box;
            box.lifetime = ros::Duration(_marker_lifetime);

            // Message headers
            box.header = in_objects.header;
            box.type = visualization_msgs::Marker::CUBE;
            box.action = visualization_msgs::Marker::ADD;
            box.ns = _marker_namespace + "/box_markers";
            box.id = _marker_id++;

            // Marker properties
            box.color = _box_color;

            box.scale.x = object.bbox.dimension.dimension.width;
            box.scale.y = object.bbox.dimension.dimension.length;
            box.scale.z = object.bbox.dimension.dimension.height;
            box.pose.position = object.bbox.pose.pose.position;
            box.pose.orientation = object.bbox.pose.pose.orientation;

            // Skip large boxes to prevent messy visualization
            if (box.scale.x > _box_max_size || box.scale.y > _box_max_size || box.scale.z > _box_max_size)
            {
                ROS_DEBUG("[%s] Object skipped with size %.2f x %.2f x %.2f!", __MODULE_NAME__
                    , box.scale.x, box.scale.y, box.scale.z);
                continue;
            }

            out_objects.markers.push_back(box);
        }
    }

    void BBoxVisualizer::addCentroids(const zzz_perception_msgs::DetectionBoxArray &in_objects, visualization_msgs::MarkerArray &out_objects)
    {
        for (auto const &object: in_objects.detections)
        {
            visualization_msgs::Marker centroid_marker;
            centroid_marker.lifetime = ros::Duration(_marker_lifetime);

            // Message headers
            centroid_marker.header = in_objects.header;
            centroid_marker.type = visualization_msgs::Marker::SPHERE;
            centroid_marker.action = visualization_msgs::Marker::ADD;
            centroid_marker.ns = _marker_namespace + "/centroid_markers";
            centroid_marker.id = _marker_id++;

            // Marker properties
            centroid_marker.scale.x = _centroid_scale;
            centroid_marker.scale.y = _centroid_scale;
            centroid_marker.scale.z = _centroid_scale;
            centroid_marker.color = _centroid_color;

            centroid_marker.pose = object.bbox.pose.pose;

            out_objects.markers.push_back(centroid_marker);
        }
    }

    void BBoxVisualizer::visualizeDetection(zzz_perception_msgs::DetectionBoxArrayConstPtr input)
    {
        visualization_msgs::MarkerArray markers;
        _marker_id = 0;

        addBoxes(*input, markers);
        addCentroids(*input, markers);
        addLabels(*input, markers);

        _output_publisher.publish(markers);
    }

    void BBoxVisualizer::visualizeTracking(zzz_perception_msgs::TrackingBoxArrayConstPtr input)
    {

    }

} } } // namespace zzz::visualization::rviz
