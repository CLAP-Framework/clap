#ifndef ZZZ_VISUALIZATION_RVIZ_DETECTION_VISUALIZER_BBOXVISUALIZER_H
#define ZZZ_VISUALIZATION_RVIZ_DETECTION_VISUALIZER_BBOXVISUALIZER_H

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <zzz_perception_msgs/DetectionBoxArray.h>
#include <zzz_perception_msgs/TrackingBoxArray.h>

namespace zzz
{
    namespace visualization
    {
        namespace rviz
        {
            enum class BBoxVisualizerType
            {
                DetectionBoxArray, // Draw detection boxes in point cloud
                DetectionBox2DArray, // Draw detection boxes in image
                TrackingBoxArray // Draw tracking boxes in point cloud
            };

            class BBoxVisualizer
            {
            public:
                BBoxVisualizer(ros::NodeHandle &node_handle, ros::NodeHandle &private_handle,
                    std::string input_topic="objects_detected", std::string output_topic="objects_visual",
                    std::string marker_namespace="bbox");
                void visualizeDetection(zzz_perception_msgs::DetectionBoxArrayConstPtr input);
                void visualizeTracking(zzz_perception_msgs::TrackingBoxArrayConstPtr input);

            protected:
                ros::Subscriber _input_subscriber;
                ros::Publisher _output_publisher;
                std::string _marker_namespace;

                std::string _frame_id; // store for debug use
                ros::Time _timestamp; // store for debug use

                BBoxVisualizerType _vis_type;
                float _marker_lifetime;
                std_msgs::ColorRGBA _centroid_color, _box_color, _label_color;
                float _centroid_scale, _label_scale;
                float _label_height;
                float _box_max_size;

                int _marker_id; // id counter
            
                void addLabels(const zzz_perception_msgs::DetectionBoxArray &in_objects, visualization_msgs::MarkerArray &out_objects);
                void addBoxes(const zzz_perception_msgs::DetectionBoxArray &in_objects, visualization_msgs::MarkerArray &out_objects);
                void addCentroids(const zzz_perception_msgs::DetectionBoxArray &in_objects, visualization_msgs::MarkerArray &out_objects);
            };
        } // namespace rviz
        
    } // namespace visualization
    
} // namespace zzz


#endif // ZZZ_VISUALIZATION_RVIZ_DETECTION_VISUALIZER_BBOXVISUALIZER_H
