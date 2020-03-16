#include <ros/ros.h> 
#include <std_msgs/Time.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/TimeReference.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geographic_msgs/GeoPointStamped.h>
#include <geodesy/utm.h>
#include <geodesy/wgs84.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl_conversions/pcl_conversions.h>   
#include <pcl/io/pcd_io.h>  
//point types
#include <pcl/point_types.h>

#include <fstream>

#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <cv.h>
#include <highgui.h>
// #include "cxcore.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <string>
// #include <sstream>
#include "obstacle_set_ros_msg.h"


#include <zzz_perception_msgs/TrackingBox.h>
#include <zzz_perception_msgs/TrackingBoxArray.h>



class imu_lidar{
public:
    imu_lidar(const ros::NodeHandle& node_handle):nh(node_handle){R_ego << 0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0; T_ego << 0.0,1.0,1.0;};
    void imu_callback(const sensor_msgs::NavSatFixConstPtr& navsat_msg, const sensor_msgs::ImuConstPtr& imu_msg){
        // ROS_INFO("--------------------imu_callback-------------------------");
        const ros::Time& gps_stamp = navsat_msg->header.stamp;
        const ros::Time& imu_stamp = imu_msg->header.stamp;

        // ROS_INFO("Time Reference: %lf, %lf", gps_stamp.toSec(), imu_stamp.toSec());
        geographic_msgs::GeoPointStampedPtr gps_msg(new geographic_msgs::GeoPointStamped());
        gps_msg->header = navsat_msg->header;
        gps_msg->position.latitude = navsat_msg->latitude;
        gps_msg->position.longitude = navsat_msg->longitude;
        gps_msg->position.altitude = navsat_msg->altitude;

        geodesy::UTMPoint utm;
        geodesy::fromMsg(gps_msg->position, utm);
        Eigen::Vector3d xyz(utm.easting, utm.northing, utm.altitude);
        T_static << xyz.x(),xyz.y(),xyz.z();
        // ROS_INFO("     xyz: %lf, %lf, %lf", xyz.x(), xyz.y(), xyz.z());
        // ROS_INFO("T_static: %lf, %lf, %lf", T_static(0), T_static(1), T_static(2));
        // std::cout<<std::endl;

        geometry_msgs::Quaternion q_orientation = imu_msg->orientation;
        Eigen::Quaterniond q(q_orientation.w, q_orientation.x, q_orientation.y, q_orientation.z);
        R_static = q.toRotationMatrix();
        // ROS_INFO("%lf, %lf, %lf ,%lf, %lf, %lf ,%lf, %lf, %lf ", R_static(0,0),R_static(0,1),
        // R_static(0,2),R_static(1,0),R_static(1,1),R_static(1,2),R_static(2,0),R_static(2,1),R_static(2,2));   
        // std::cout<<"R_ego:"<<std::endl<<R_ego<<std::endl;
        // std::cout<<"T_ego:"<<std::endl<<T_ego<<std::endl;

        // std::cout<<"R_static:"<<std::endl<<R_static<<std::endl;
        // std::cout<<"T_static:"<<std::endl<<T_static<<std::endl;
        // imu_lidar::show(xyz.x(), xyz.y());
    }

    void gps_vel_callback(const geometry_msgs::TwistWithCovarianceStamped& vel_msg){
        // ROS_INFO("++++++++++++++++++++gps_vel_callback+++++++++++++++++++");
        // std::cout<<"x: ";
        // std::cout<<vel_msg.twist.twist.linear.x<<std::endl;
        // std::cout<<"y: ";
        // std::cout<<vel_msg.twist.twist.linear.y<<std::endl;
        // std::cout<<"z: ";
        // std::cout<<vel_msg.twist.twist.linear.z<<std::endl;
        vel_static << vel_msg.twist.twist.linear.x, vel_msg.twist.twist.linear.y, vel_msg.twist.twist.linear.z;
        // std::cout<<vel_static<<std::endl<<std::endl;
    }


    void obs_callback(const rs_perception::obstacle_set_ros_msg& obs_msg){
        // ROS_INFO("***** obs_callback %d", obs_msg.obstcles.size());
        // std::cout << obs_msg.obstcles[1].geo_center.x<<std::endl;
        cv::Mat tmp_disp = cv::Mat(1200, 1200, CV_8UC3, cv::Scalar(255,255,255));
        obs_msg_static = obs_msg;
        int scale_vel = 5;

        zzz_perception_msgs::TrackingBoxArray obs_array;
        obs_array.header.stamp = ros::Time::now();
        obs_array.header.frame_id = "map";
        for (int i = 0; i < obs_msg.obstcles.size(); i++) {
            // int i = 0;
            //std::cout <<"-------- " << i << "--" << obs_msg.obstcles[i].tracker_id << " -----------: " << obs_msg.obstcles[i].geo_center.x << " " << obs_msg.obstcles[i].geo_center.y << " " << obs_msg.obstcles[i].geo_center.z << " " <<std::endl;
            Eigen::Vector3d old_geo_center, new_geo_center;
            old_geo_center << obs_msg.obstcles[i].geo_center.x, obs_msg.obstcles[i].geo_center.y, obs_msg.obstcles[i].geo_center.z;
            //std::cout<<"old_geo_center:"<<std::endl<<old_geo_center<<std::endl;
            new_geo_center = R_static*(R_ego*old_geo_center+T_ego)+T_static;
            obs_msg_static.obstcles[i].geo_center.x = new_geo_center(0);
            obs_msg_static.obstcles[i].geo_center.y = new_geo_center(1);
            obs_msg_static.obstcles[i].geo_center.z = new_geo_center(2);

            Eigen::Vector3d old_geo_direction, new_geo_direction;
            old_geo_direction << obs_msg.obstcles[i].geo_direction.x, obs_msg.obstcles[i].geo_direction.y, obs_msg.obstcles[i].geo_direction.z;
            new_geo_direction = R_static*R_ego*old_geo_direction;
            obs_msg_static.obstcles[i].geo_direction.x = new_geo_direction(0);
            obs_msg_static.obstcles[i].geo_direction.y = new_geo_direction(1);
            obs_msg_static.obstcles[i].geo_direction.z = new_geo_direction(2);

            // std::cout<<"100: "<<std::endl<<obs_msg_static.obstcles[i].geo_direction.x<<std::endl<<obs_msg_static.obstcles[i].geo_direction.y<<std::endl<<obs_msg_static.obstcles[i].geo_direction.z<<std::endl;

            Eigen::Vector3d old_vel, new_vel;
            old_vel << obs_msg.obstcles[i].velocity.x, obs_msg.obstcles[i].velocity.y, obs_msg.obstcles[i].velocity.z;
            new_vel = R_static*R_ego*old_vel;
            obs_msg_static.obstcles[i].velocity.x = new_vel(0) + vel_static(0);
            obs_msg_static.obstcles[i].velocity.y = new_vel(1) + vel_static(1);
            obs_msg_static.obstcles[i].velocity.z = new_vel(2) + vel_static(2);

            Eigen::Vector3d old_ave_vel, new_ave_vel;
            old_ave_vel << obs_msg.obstcles[i].ave_velocity.x, obs_msg.obstcles[i].ave_velocity.y, obs_msg.obstcles[i].ave_velocity.z;
            new_ave_vel = R_static*R_ego*old_ave_vel;
            obs_msg_static.obstcles[i].ave_velocity.x = new_ave_vel(0) + vel_static(0);
            obs_msg_static.obstcles[i].ave_velocity.y = new_ave_vel(1) + vel_static(1);
            obs_msg_static.obstcles[i].ave_velocity.z = new_ave_vel(2) + vel_static(2);


            // add surrounding obstacles orientation information.
            Eigen::Matrix3d old_geo_dir_matrix, new_geo_dir_matrix;
            float cos_theta = old_geo_direction(0) / sqrt(old_geo_direction(0)*old_geo_direction(0) + old_geo_direction(1)*old_geo_direction(1));
            float sin_theta = old_geo_direction(1) / sqrt(old_geo_direction(0)*old_geo_direction(0) + old_geo_direction(1)*old_geo_direction(1));
            old_geo_dir_matrix << cos_theta, -sin_theta, 0, sin_theta, cos_theta, 0, 0, 0, 1;
            new_geo_dir_matrix << R_static*R_ego*old_geo_dir_matrix;
            Eigen::Quaterniond geo_dir_quat(new_geo_dir_matrix);

            // cv::Point p1(int(obs_msg.obstcles[i].geo_center.x*5+600),int(obs_msg.obstcles[i].geo_center.y*5+600));
            // cv::Point p2(int((obs_msg_static.obstcles[i].geo_center.x-442860)*5+600),int((obs_msg_static.obstcles[i].geo_center.y-4427880)*5+600));
            
            // cv::Point p3(int((obs_msg_static.obstcles[i].geo_center.x-442860)*5+600 + scale_vel*obs_msg_static.obstcles[i].geo_direction.x),int((obs_msg_static.obstcles[i].geo_center.y-4427880)*5 + 600+scale_vel*obs_msg_static.obstcles[i].geo_direction.y));
            // cv::Point p4(int((obs_msg_static.obstcles[i].geo_center.x-442860)*5+600 + scale_vel*obs_msg_static.obstcles[i].velocity.x),int((obs_msg_static.obstcles[i].geo_center.y-4427880)*5 + 600+scale_vel*obs_msg_static.obstcles[i].velocity.y));

            // //cv::circle(tmp_disp, p1, 4, cv::Scalar(0, 0, 255), -1);
            // cv::circle(tmp_disp, p2, 4, cv::Scalar(0, 0, 0), -1);
            // //cv::arrowedLine(tmp_disp, p2, p3, cv::Scalar(125, 125, 125));
            // cv::arrowedLine(tmp_disp, p2, p4, cv::Scalar(0, 0, 0), 2);

            zzz_perception_msgs::TrackingBox obs_box;
            // TODO 
            zzz_perception_msgs::ObjectClass t;
            if (obs_msg_static.obstcles[i].type == 3 || 
                obs_msg_static.obstcles[i].type == 4) {
                t.classid = 1;
            } else {
                t.classid = 2;
            }
            obs_box.classes.push_back(t);
            obs_box.classes[0].score = obs_msg_static.obstcles[i].type_confidence;

            obs_box.uid = obs_msg_static.obstcles[i].id;
            obs_box.confidence = 1.0;

            // pose
            // TODO (map origin for temp 442867, 4427888)
            obs_box.bbox.pose.pose.position.x = obs_msg_static.obstcles[i].geo_center.x - 442867;
            obs_box.bbox.pose.pose.position.y = obs_msg_static.obstcles[i].geo_center.y - 4427888;
            obs_box.bbox.pose.pose.position.z = obs_msg_static.obstcles[i].geo_center.z;
            // orientation
            obs_box.bbox.pose.pose.orientation.x = geo_dir_quat.x();
            obs_box.bbox.pose.pose.orientation.y = geo_dir_quat.y();
            obs_box.bbox.pose.pose.orientation.z = geo_dir_quat.z();
            obs_box.bbox.pose.pose.orientation.w = geo_dir_quat.w();


            // TODO default value should be changed.
            obs_box.bbox.dimension.length_x = 4;
            obs_box.bbox.dimension.length_y = 2;
            obs_box.bbox.dimension.length_z = 1.8;
            // twist
            obs_box.twist.twist.linear.x = obs_msg_static.obstcles[i].velocity.x;
            obs_box.twist.twist.linear.y = obs_msg_static.obstcles[i].velocity.y;
            obs_box.twist.twist.linear.z = obs_msg_static.obstcles[i].velocity.z;


            obs_array.targets.push_back(obs_box);
        }
        // ROS_INFO("##### obs_array length - %d", obs_array.targets.size());
        obs_pub_.publish(obs_array);
        // cv::Point p3(int((T_static(0)-442860)*5+600),int((T_static(1)-4427880)*5+600));
        // cv::Point p5(int((T_static(0)-442860)*5+600+scale_vel*vel_static(0)),int((T_static(1)-4427880)*5+600+scale_vel*vel_static(1)));
        // cv::circle(tmp_disp, p3, 8, cv::Scalar(0, 0, 255), -1);
        // cv::arrowedLine(tmp_disp, p3, p5, cv::Scalar(0, 0, 0),2);
        // cv::imshow("disp", tmp_disp);
        // cv::waitKey(0);
    }

    void run() {
        gps_fix_sub.reset(new message_filters::Subscriber<sensor_msgs::NavSatFix>(nh, "/gps/fix", 200));
        imu_sub.reset(new message_filters::Subscriber<sensor_msgs::Imu>(nh, "/imu/data", 200));
        sync.reset(new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), *gps_fix_sub, *imu_sub));
        sync->registerCallback(boost::bind(&imu_lidar::imu_callback, this, _1, _2));
        obs_sub = nh.subscribe("/rs_obstacle", 1, &imu_lidar::obs_callback,this);
        gps_vel_sub = nh.subscribe("/gps/vel", 200, &imu_lidar::gps_vel_callback,this);
        obs_pub_ = nh.advertise<zzz_perception_msgs::TrackingBoxArray>("zzz/perception/objects_tracked", 2);
    }

    void show(int x, int y) {
        cv::Point p(int(5*(x-442860+40)),int(5*(y-4427880+20)));
        cv::circle(disp, p, 1, cv::Scalar(255, 255, 255), -1);
        cv::imshow("disp", disp);
        cv::waitKey(0);
    }

    ros::NodeHandle nh;

    std::unique_ptr<message_filters::Subscriber<sensor_msgs::NavSatFix>> gps_fix_sub;
    std::unique_ptr<message_filters::Subscriber<sensor_msgs::Imu>> imu_sub;
    
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::NavSatFix, sensor_msgs::Imu> MySyncPolicy;
    std::unique_ptr<message_filters::Synchronizer<MySyncPolicy>> sync;
    ros::Subscriber obs_sub, gps_vel_sub;

    // topic : /zzz/perception/objects_tracked
    ros::Publisher obs_pub_; 

    cv::Mat disp = cv::Mat::zeros(800, 800, CV_8UC3);

    Eigen::Matrix3d R_static; 
    Eigen::Vector3d T_static;

    Eigen::Matrix3d R_ego;
    Eigen::Vector3d T_ego;

    Eigen::Vector3d vel_static;

    rs_perception::obstacle_set_ros_msg obs_msg_static;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_gps");     
    ros::NodeHandle nh;  
    imu_lidar il(nh);
    il.run();
    // il.show();
    // ros::Subscriber obs_sub = nh.subscribe("/rs_obstacle", 1, obs_callback);

    ros::spin();

    return 0;
}