#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <zzz_perception_msgs/DetectionBoxArray.h>

#include <pcl/conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <Eigen/Core>
#include <Eigen/SVD>

class Detector
{
protected:
    ros::NodeHandle _nh;
    ros::Subscriber _input;
    ros::Publisher _output;

private:
    typedef pcl::PointXYZ PointType;
    typedef pcl::PointCloud<PointType> PointCloudType;
    void estimate(const PointCloudType::Ptr object, zzz_perception_msgs::DetectionBox &bound)
    {
        pcl::toROSMsg(*object, bound.source_cloud);
        
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid (*object, centroid);
        bound.bbox.pose.pose.position.x = centroid(0);
        bound.bbox.pose.pose.position.y = centroid(1);
        bound.bbox.pose.pose.position.z = centroid(2);
        
        Eigen::MatrixXf mat = object->getMatrixXfMap();
        Eigen::MatrixXf xy = mat.leftCols(2);
        xy.col(0).array() -= centroid(0);
        xy.col(1).array() -= centroid(1);
        Eigen::JacobiSVD<Eigen::MatrixXf> svd(xy, Eigen::ComputeFullV);
        Eigen::Matrix2f v = svd.matrixV().transpose();
        float l = (xy*v.col(0)).maxCoeff() - (xy*v.col(0)).minCoeff();
        float w = (xy*v.col(1)).maxCoeff() - (xy*v.col(1)).minCoeff();
        float h = mat.col(2).maxCoeff() - mat.col(2).minCoeff();
        bound.bbox.dimension.dimension.length = l;
        bound.bbox.dimension.dimension.width = w;
        bound.bbox.dimension.dimension.height = h;
    }

protected:
    void detect(const sensor_msgs::PointCloud2 &input)
    {
        // Read in the cloud data
        PointCloudType::Ptr cloud (new PointCloudType), cloud_f (new PointCloudType);
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(input, pcl_pc2);
        pcl::fromPCLPointCloud2(pcl_pc2,*cloud);
        ROS_INFO("PointCloud Received");

        // Create the segmentation object for the planar model and set all the parameters
        pcl::SACSegmentation<PointType> seg;
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        seg.setOptimizeCoefficients (true);
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setMaxIterations (100);
        seg.setDistanceThreshold (0.02);

        PointCloudType::Ptr cloud_filtered (new PointCloudType(*cloud));
        int i=0, nr_points = (int) cloud->points.size ();
        while (true)
        {
            // Segment the largest planar component from the remaining cloud
            seg.setInputCloud (cloud_filtered);
            seg.segment (*inliers, *coefficients);
            if (inliers->indices.size () == 0)
            {
                ROS_ERROR("Could not estimate a planar model for the given dataset.");
                break;
            }

            // Extract the planar inliers from the input cloud
            pcl::ExtractIndices<PointType> extract;
            extract.setInputCloud (cloud_filtered);
            extract.setIndices (inliers);
            extract.setNegative (false);

            // Remove the planar inliers, extract the rest
            extract.setNegative (true);
            extract.filter (*cloud_f);
            *cloud_filtered = *cloud_f;

            // Judge the points associated with the planar surface
            ROS_INFO_STREAM("PointCloud representing the planar component: " << inliers->indices.size () << " data points.");
            if(inliers->indices.size () < 0.1 * nr_points) break;
        }
        ROS_INFO_STREAM("PointCloud representing the objects: " << cloud_filtered->size () << " data points.");

        // Creating the KdTree object for the search method of the extraction
        pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>);
        tree->setInputCloud (cloud_filtered);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<PointType> ec;
        ec.setClusterTolerance (0.05); // m
        ec.setMinClusterSize (100);
        ec.setMaxClusterSize (25000);
        ec.setSearchMethod (tree);
        ec.setInputCloud (cloud_filtered);
        ec.extract (cluster_indices);
        ROS_INFO_STREAM("Objects: " << cluster_indices.size ());

        int j = 0; zzz_perception_msgs::DetectionBox dobj; zzz_perception_msgs::DetectionBoxArray dobjects;
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
        {
            PointCloudType::Ptr cloud_cluster (new PointCloudType);
            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
                cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
            cloud_cluster->width = cloud_cluster->points.size ();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;

            ROS_INFO_STREAM("PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points.");
            estimate(cloud_cluster, dobj);
            dobjects.detections.push_back(dobj);
            j++;
        }
        _output.publish(dobjects);
    }

public:
    Detector(ros::NodeHandle nh, std::string in_name, std::string out_name) : _nh(nh)
    {
        _input = _nh.subscribe(in_name, 1, &Detector::detect, this);
        _output = _nh.advertise<zzz_perception_msgs::DetectionBoxArray>(out_name, 1);
    }
};


int main (int argc, char** argv)
{
    ros::init(argc, argv, "simple_detector_node");
    ros::NodeHandle n; Detector detector(n, "/kitti/velo/pointcloud", "simple_detector_objs");
    ros::spin();
}