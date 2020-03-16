#define __MODULE_NAME__ "EuclideanClusterDetector"
#include "zzz_perception_detection_lidar_detectors/EuclideanClusterDetector.h"

#include <unordered_set>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

using namespace std;
using namespace pcl;

namespace zzz { namespace perception {
    EuclideanClusterDetector::EuclideanClusterDetector(ros::NodeHandle &nh, ros::NodeHandle &pnh)
        : LidarDetector(nh, pnh)
    {
        #ifndef NDEBUG
        string cluster_topic, plane_topic;
        pnh.param("cluster_topic", cluster_topic, string("points_cluster"));
        pnh.param("plane_topic", plane_topic, string("points_plane"));
        _cluster_publisher = nh.advertise<sensor_msgs::PointCloud2>(cluster_topic, 1);
        _plane_publisher = nh.advertise<sensor_msgs::PointCloud2>(plane_topic, 1);
        #endif

        // setup parameters
        pnh.param("downsampling_enable", _downsampling_enable, false);
        pnh.param("downsampling_leaf_size", _downsampling_leaf_size, 0.1f);

        pnh.param("crop_enable", _crop_enable, false);
        pnh.param("plane_removal_enable", _plane_removal_enable, true);
        pnh.param("plane_removal_ransac_thres", _plane_removal_ransac_thres, 0.15f);
        pnh.param("plane_removal_count_thres", _plane_removal_count_thres, 0.1f);

        pnh.param("cluster_dist_thres", _cluster_dist_thres, 0.5f);
        pnh.param("cluster_count_min", _cluster_count_min, 100);
        pnh.param("cluster_count_max", _cluster_count_max, 1000000);
    }

    PointIndices::Ptr EuclideanClusterDetector::planeRemove(
        PointCloud<PointXYZ>::ConstPtr input, PointIndices::ConstPtr indices)
    {
        // Setup used variables
        PointIndices::ConstPtr indices_input_const = indices;
        PointIndices::Ptr indices_output, indices_temp;
        PointCloud<PointXYZ>::ConstPtr cloud_input = input;
        #ifndef NDEBUG
        PointCloud<PointXYZL>::Ptr cloud_plane(new PointCloud<PointXYZL>);
        #endif

        // Setup plane segmenter
        SACSegmentation<PointXYZ> seg;
        PointIndices::Ptr inliers (new PointIndices);
        ModelCoefficients::Ptr coefficients (new ModelCoefficients);
        seg.setOptimizeCoefficients (true); // note: not taking too much time
        seg.setModelType (SACMODEL_PLANE);
        seg.setMethodType (SAC_RANSAC);
        seg.setMaxIterations (100);
        seg.setDistanceThreshold (_plane_removal_ransac_thres);

        // Remove plane recursively
        int points_count = indices_input_const->indices.size (), planes_count = 0;
        while (true)
        {
            // Segment the largest planar component from the remaining cloud
            seg.setInputCloud (cloud_input);
            seg.setIndices(indices_input_const);
            seg.segment (*inliers, *coefficients);
            if (inliers->indices.size () == 0)
            {
                ROS_ERROR("[%s] Could not estimate a planar model for the point cloud.", __MODULE_NAME__);
                break;
            }

            // Judge the points associated with the planar surface
            ROS_DEBUG_STREAM("[" << __MODULE_NAME__ << "] PointCloud representing the planar component: "
                                 << inliers->indices.size () << " data points.");
            if(inliers->indices.size () < _plane_removal_count_thres * points_count) break;

            // Extract the planar inliers from the input cloud
            unordered_set<int> indices_set(inliers->indices.begin(), inliers->indices.end());
            indices_temp = PointIndices::Ptr(new PointIndices);
            indices_temp->indices.reserve(indices_set.size());
            for (auto iter = indices_input_const->indices.begin(); iter != indices_input_const->indices.end(); iter++)
            {
                if (indices_set.find(*iter) == indices_set.end())
                    // Add non-plane points to output indices
                    indices_temp->indices.push_back(*iter);
                #ifndef NDEBUG
                else // Add plane points to plane cloud
                {
                    PointXYZL npoint;
                    npoint.x = cloud_input->at(*iter).x;
                    npoint.y = cloud_input->at(*iter).y;
                    npoint.z = cloud_input->at(*iter).z;
                    npoint.label = planes_count;
                    cloud_plane->push_back(npoint);
                }
                #endif
            }
            indices_input_const = indices_output = indices_temp;
            planes_count++;
        }
        
        #ifndef NDEBUG
        // Publish ground
        sensor_msgs::PointCloud2Ptr msg(new sensor_msgs::PointCloud2);
        toROSMsg(*cloud_plane, *msg);
        msg->header.frame_id = _frame_id;
        msg->header.stamp = _timestamp;
        _plane_publisher.publish(msg);
        #endif

        // If no plane is found
        if (indices_output == nullptr)
        {
            *indices_output = *indices_input_const;
            ROS_WARN("[%s] Could not find planes in point cloud.", __MODULE_NAME__);
        }

        return indices_output;
    }

    void EuclideanClusterDetector::segment(PointCloud<PointXYZ>::ConstPtr input,
        PointIndices::ConstPtr indices, std::vector<pcl::PointIndices> &clusters)
    {
        pcl::search::KdTree<PointXYZ>::Ptr tree (new pcl::search::KdTree<PointXYZ>);
        boost::shared_ptr<vector<int>> tree_indices(new vector<int>(indices->indices));
        tree->setInputCloud (input, tree_indices);

        EuclideanClusterExtraction<PointXYZ> ec;
        ec.setClusterTolerance (_cluster_dist_thres); // m
        ec.setMinClusterSize (_cluster_count_min);
        ec.setMaxClusterSize (_cluster_count_max);
        ec.setSearchMethod (tree);
        ec.setInputCloud (input);
        ec.setIndices (indices);
        ec.extract (clusters);
        ROS_DEBUG("[%s] Number of objects: %d", __MODULE_NAME__, (int)clusters.size());
    }

    void EuclideanClusterDetector::estimate(PointCloud<pcl::PointXYZ>::ConstPtr input,
        zzz_perception_msgs::BoundingBox &bbox)
    {        
        // Estimate centroid
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid (*input, centroid);
        bbox.pose.pose.position.x = centroid(0);
        bbox.pose.pose.position.y = centroid(1);
        bbox.pose.pose.position.z = centroid(2);
        
        // Naive size estimation
        PointXYZ min_pt, max_pt;
        getMinMax3D(*input, min_pt, max_pt);
        bbox.dimension.length_x = max_pt.x - min_pt.x;
        bbox.dimension.length_y  = max_pt.y - min_pt.y;
        bbox.dimension.length_z = max_pt.z - min_pt.z;
    }

    void EuclideanClusterDetector::detect(sensor_msgs::PointCloud2ConstPtr input)
    {
        // Parse input
        _frame_id = input->header.frame_id;
        _timestamp = input->header.stamp;
        PCLPointCloud2Ptr in_cloud2(new PCLPointCloud2);
        PointCloud<PointXYZ>::Ptr in_cloud (new PointCloud<PointXYZ>);
        pcl_conversions::toPCL(*input, *in_cloud2);
        fromPCLPointCloud2(*in_cloud2, *in_cloud);
        ROS_DEBUG("[%s] PointCloud Received (size: %d)", __MODULE_NAME__, (int)in_cloud->size());
        
        PointIndices::Ptr indices_output(new PointIndices);
        indices_output->indices.resize(in_cloud->size());
        iota(indices_output->indices.begin(), indices_output->indices.end(), 0);
        
        // Crop
        if (_crop_enable)
            ROS_WARN("[%s] Crop is not supported yet. It will not be performed", __MODULE_NAME__);

        // Downsampling
        if (_downsampling_enable)
            ROS_WARN("[%s] Downsampling is not supported yet. It will not be performed", __MODULE_NAME__);

        // Plane Removal
        if (_plane_removal_enable)
            indices_output = planeRemove(in_cloud, indices_output);
        
        // Segmentation
        vector<PointIndices> cluster_indices;
        zzz_perception_msgs::DetectionBoxArray array;
        segment(in_cloud, indices_output, cluster_indices);

        // Pose estimation
        #ifndef NDEBUG
        PointCloud<PointXYZL>::Ptr aggr_cluster_cloud(new PointCloud<PointXYZL>);
        #endif
        for (int i = 0; i < cluster_indices.size(); i++)
        {
            ROS_DEBUG("[%s] PointCloud representing the Cluster: %d data points.", __MODULE_NAME__, (int)cluster_indices[i].indices.size());
            zzz_perception_msgs::DetectionBox detection;

            // Select points for estimation and debug
            PointCloud<PointXYZ>::Ptr cluster_cloud(new PointCloud<PointXYZ>);
            cluster_cloud->reserve(cluster_indices[i].indices.size());
            for (auto pit = cluster_indices[i].indices.begin (); pit != cluster_indices[i].indices.end (); ++pit)
            {
                cluster_cloud->points.push_back (in_cloud->points[*pit]);
                #ifndef NDEBUG
                PointXYZL npoint;
                npoint.x = in_cloud->at(*pit).x;
                npoint.y = in_cloud->at(*pit).y;
                npoint.z = in_cloud->at(*pit).z;
                npoint.label = i;
                aggr_cluster_cloud->push_back(npoint);
                #endif
            }
            estimate(cluster_cloud, detection.bbox);

            // Select points for message
            PCLPointCloud2 out_cloud2;
            boost::shared_ptr<PointIndices> indices_ptr(new PointIndices(cluster_indices[i]));
            ExtractIndices<PCLPointCloud2> extract;
            extract.setInputCloud(in_cloud2);
            extract.setIndices(indices_ptr);
            extract.filter(out_cloud2);
            pcl_conversions::fromPCL(out_cloud2, detection.source_cloud);

            array.detections.push_back(detection);
        }

        #ifndef NDEBUG
        // Publish ground
        sensor_msgs::PointCloud2Ptr msg(new sensor_msgs::PointCloud2);
        toROSMsg(*aggr_cluster_cloud, *msg);
        msg->header.frame_id = _frame_id;
        msg->header.stamp = _timestamp;
        _cluster_publisher.publish(msg);
        #endif

        // Publish detections
        array.header.frame_id = _frame_id;
        array.header.stamp = _timestamp;
        _output_publisher.publish(array);
    }
}} // namespace zzz::perception

