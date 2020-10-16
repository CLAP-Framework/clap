/*
 * @Author: jiang xiao xu 
 * @Date: 2020-05-08 14:45:16
 * @LastEditors: jiang xiao xu
 * @LastEditTime: 2020-05-19 15:07:39
 * @Description:   this fusion is for oxford   car Coordinate system: x_forword  y_left  z_up
 */
#include "ros/ros.h"
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/TimeReference.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>

#define __ZZZ_SUPPORT__ 1

#if __ZZZ_SUPPORT__
#include <zzz_driver_msgs/RigidBodyState.h>
#include <zzz_driver_msgs/RigidBodyStateStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#endif

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <stdio.h>
#include <fstream>
#include <queue>
#include <mutex>
#include <thread>
#include <geodesy/utm.h>
#include <geodesy/wgs84.h>
#include <geographic_msgs/GeoPointStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// UTM conversion
#include <gps_common/conversions.h>
using namespace std;

class global_fusion
{
private:
    ros::NodeHandle n;
    //sub
    ros::Subscriber sub_fix;
    ros::Subscriber sub_vel;
    ros::Subscriber sub_imu;
    ros::Subscriber sub_odom;
    ros::Subscriber sub_channel;
    ros::Subscriber sub_pos_type;
    ros::Subscriber sub_nav_status;
    ros::Subscriber sub_gps_time_ref;
    ros::Subscriber sub_gga;

    //pub
    ros::Publisher pub_fix;
    ros::Publisher pub_vel;
    ros::Publisher pub_imu;
    ros::Publisher pub_odom;
    //ros::Publisher pub_channel = n.ad
    ros::Publisher pub_pos_type;
    ros::Publisher pub_nav_status;
    ros::Publisher pub_gps_time_ref;
    ros::Publisher pub_gga;

    ros::Publisher pub_car;
    ros::Publisher pub_Groundtruth_path; //Groundtruth 轨迹
    ros::Publisher pub_ego_pose;

    //path
    nav_msgs::Path Groundtruth_path;

    //mutex
    std::mutex imu_mutex;
    std::mutex gps_mutex;
    std::mutex vel_mutex;
    std::mutex odom_mutex;

    // in topics name
    std::string in_imu_topic;
    std::string in_gps_topic;
    std::string in_vel_topic;
    std::string in_odom_topic;
    std::string in_pos_type_topic;
    std::string in_nav_status_topic;
    std::string in_gps_time_ref_topic;

    // out topics name
    std::string gps_imu_topic;
    std::string gps_fix_topic;
    std::string gps_vel_topic;
    std::string gps_odom_topic;
    std::string gps_pos_type_topic;
    std::string gps_nav_status_topic;
    std::string gps_time_ref_topic;

    //utm0
    double utm0_x;
    double utm0_y;
    double lat0;
    double lon0;
    double alt0;

    //debug
    bool debug;
    bool yaw_n2w;

public:
    sensor_msgs::Imu new_imu_msg;
    geometry_msgs::TwistWithCovarianceStamped new_twist;
    nav_msgs::Odometry new_odom;

public:
    global_fusion()
    {
        //utm0_x = 0;
        //utm0_y = 0;
        lat0 = 0;
        lon0 = 0;
        alt0 = 0;
        debug = false;
        yaw_n2w = false;
        //in
        n.getParam("in_gps_topic", in_gps_topic);
        n.getParam("in_imu_topic", in_imu_topic);

        n.getParam("in_vel_topic", in_vel_topic);
        n.getParam("in_odom_topic", in_odom_topic);
        n.getParam("in_pos_type_topic", in_pos_type_topic);
        n.getParam("in_nav_status_topic", in_nav_status_topic);
        n.getParam("in_gps_time_ref_topic", in_gps_time_ref_topic);
        //n.getParam("utm0_x", utm0_x);
        //n.getParam("utm0_y", utm0_y);
        n.getParam("lat0", lat0);
        n.getParam("lon0", lon0);
        n.getParam("alt0", alt0);

        //out
        n.getParam("gps_fix_topic", gps_fix_topic);
        n.getParam("gps_imu_topic", gps_imu_topic);
        n.getParam("gps_vel_topic", gps_vel_topic);
        n.getParam("gps_odom_topic", gps_odom_topic);
        n.getParam("gps_pos_type_topic", gps_pos_type_topic);
        n.getParam("gps_nav_status_topic", gps_nav_status_topic);
        n.getParam("gps_time_ref_topic", gps_time_ref_topic);

        //debug
        n.getParam("debug", debug);
        n.getParam("yaw_n2w", yaw_n2w);
        ROS_INFO("--OUTPUT MESSAGE--");
        ROS_INFO("\n%s\n%s\n%s\n%s\n%s\n%s\n%s\n", gps_fix_topic.c_str(),
                 gps_imu_topic.c_str(),
                 gps_vel_topic.c_str(),
                 gps_odom_topic.c_str(),
                 gps_pos_type_topic.c_str(),
                 gps_nav_status_topic.c_str(),
                 gps_time_ref_topic.c_str());

        NodeSetup();
    }
    ~global_fusion() {}
    /**
     * 
    **/
    void NodeSetup()
    {
        //subscribe
        sub_fix = n.subscribe(in_gps_topic, 100, &global_fusion::fix_callback, this);
        sub_imu = n.subscribe(in_imu_topic, 100, &global_fusion::imu_callback, this);
        sub_vel = n.subscribe(in_vel_topic, 100, &global_fusion::vel_callback, this);
        sub_odom = n.subscribe(in_odom_topic, 100, &global_fusion::odom_callback, this);

        sub_pos_type = n.subscribe(in_pos_type_topic, 100, &global_fusion::pos_type_callback, this);
        sub_nav_status = n.subscribe(in_nav_status_topic, 100, &global_fusion::nav_status_callback, this);
        sub_gps_time_ref = n.subscribe(in_gps_time_ref_topic, 100, &global_fusion::gps_time_ref_callback, this);

        // //Publisher
        pub_fix = n.advertise<sensor_msgs::NavSatFix>(gps_fix_topic, 2);
        pub_vel = n.advertise<geometry_msgs::TwistWithCovarianceStamped>(gps_vel_topic, 2);
        pub_imu = n.advertise<sensor_msgs::Imu>(gps_imu_topic, 2);
        pub_odom = n.advertise<nav_msgs::Odometry>(gps_odom_topic, 2);
        //pub_channel = n.advertise<oxford_gps_msgs::Channel>("gps/channel", 2);
        pub_pos_type = n.advertise<std_msgs::String>(gps_pos_type_topic, 2);
        pub_nav_status = n.advertise<std_msgs::String>(gps_nav_status_topic, 2);
        pub_gps_time_ref = n.advertise<sensor_msgs::TimeReference>(gps_time_ref_topic, 2);

        pub_car = n.advertise<visualization_msgs::MarkerArray>("car_model", 20);
        pub_Groundtruth_path = n.advertise<nav_msgs::Path>("groundtruth_path", 20);
        pub_ego_pose = n.advertise<zzz_driver_msgs::RigidBodyStateStamped>("zzz/navigation/ego_pose", 2);
    }
    /*
     TODO:att to q
    */
    void att2q(const double roll, const double pitch, const double yaw,
               double *w, double *x, double *y, double *z)
    {
        double roll2 = roll / 2;
        double pitch2 = pitch / 2;
        double yaw2 = yaw / 2;
        double sin_roll2 = sin(roll2);
        double sin_pitch2 = sin(pitch2);
        double sin_yaw2 = sin(yaw2);
        double cos_roll2 = cos(roll2);
        double cos_pitch2 = cos(pitch2);
        double cos_yaw2 = cos(yaw2);
        double sp = sin_pitch2, sr = sin_roll2, sy = sin_yaw2;
        double cp = cos_pitch2, cr = cos_roll2, cy = cos_yaw2;

        *w = cp * cr * cy - sp * sr * sy;
        *x = sp * cr * cy - cp * sr * sy;
        *y = cp * sr * cy + sp * cr * sy;
        *z = cp * cr * sy + sp * sr * cy;
    }

    void imu_callback(const sensor_msgs::ImuPtr &Imu_msg)
    {
        //pub_imu.publish(Imu_msg);
        //when get msg, sent immediately
        std::lock_guard<std::mutex> lock(imu_mutex);
        Eigen::Quaterniond q(Imu_msg->orientation.w, Imu_msg->orientation.x, Imu_msg->orientation.y, Imu_msg->orientation.z);

        double roll, pitch, yaw;
        tf::Matrix3x3(tf::Quaternion(Imu_msg->orientation.x, Imu_msg->orientation.y, Imu_msg->orientation.z, Imu_msg->orientation.w)).getRPY(roll, pitch, yaw);
        if (debug)
            std::cout << "imu 欧拉角roll：" << roll * 180 / 3.1415f << ",pitch:" << pitch * 180 / 3.1415f << ",yaw:" << yaw * 180 / 3.1415f << std::endl; // Q = q;

        // yaw = M_PI_2 - yaw; // ENU
        if (yaw < 0)
            yaw = yaw + 2 * M_PI; //-180~180到 0-360

        if (yaw_n2w)
        {
            std::cout << "yaw_n2w" << std::endl;
            yaw = 2 * M_PI - yaw; //yaw 逆时针 0-360度
        }

        if (debug)
            std::cout << "============================================================>after 欧拉角roll：" << roll * 180 / 3.1415f << ",pitch:" << pitch * 180 / 3.1415f << ",yaw:" << yaw * 180 / 3.1415f << std::endl; // Q = q;
    
        new_imu_msg.header = Imu_msg->header;
        double W = 0, X = 0, Y = 0, Z = 0;
        att2q(roll, pitch, yaw, &W, &X, &Y, &Z);

        if (debug)
            std::cout << "XYZW=" << W << "," << X << "," << Y << "," << Z << std::endl;
    
        new_imu_msg.orientation.w = W;
        new_imu_msg.orientation.x = X;
        new_imu_msg.orientation.y = Y;
        new_imu_msg.orientation.z = Z;
        new_imu_msg.orientation_covariance = Imu_msg->orientation_covariance;

        //牛津本来x为前, 无需调换; y方向由于牛津y本来朝右,现在改为朝左填符号
        new_imu_msg.linear_acceleration.x = Imu_msg->linear_acceleration.x;
        new_imu_msg.linear_acceleration.y = -1 * Imu_msg->linear_acceleration.y;
        new_imu_msg.linear_acceleration.z = Imu_msg->linear_acceleration.z;
        new_imu_msg.linear_acceleration_covariance[0] = Imu_msg->linear_acceleration_covariance[0];
        new_imu_msg.linear_acceleration_covariance[4] = Imu_msg->linear_acceleration_covariance[4];
        new_imu_msg.linear_acceleration_covariance[8] = Imu_msg->linear_acceleration_covariance[8];

        new_imu_msg.angular_velocity.x = Imu_msg->angular_velocity.x;
        new_imu_msg.angular_velocity.y = -1 * Imu_msg->angular_velocity.y;
        new_imu_msg.angular_velocity.z = Imu_msg->angular_velocity.z;

        new_imu_msg.angular_velocity_covariance[0] = Imu_msg->angular_velocity_covariance[0];
        new_imu_msg.angular_velocity_covariance[4] = Imu_msg->angular_velocity_covariance[4];
        new_imu_msg.angular_velocity_covariance[8] = Imu_msg->angular_velocity_covariance[8];

        pub_imu.publish(new_imu_msg);
    }

    void fix_callback(const sensor_msgs::NavSatFixConstPtr &Gps_msg)
    {
        pub_fix.publish(Gps_msg);
    }

    void vel_callback(const geometry_msgs::TwistWithCovarianceStamped &Vel_msg)
    {
        new_twist = Vel_msg;
        pub_vel.publish(new_twist);
    }

    void odom_callback(const nav_msgs::OdometryConstPtr &Odom_msg)
    {
        double UTMX0;
        double UTMY0;
        std::string utm0_zone;
        gps_common::LLtoUTM(lat0, lon0, UTMY0, UTMX0, utm0_zone);

        // shougang map origin  (428191,4417667)
        UTMX0 = 428191;
        UTMY0 = 4417667;
        new_odom.header = Odom_msg->header;
        new_odom.pose.pose.orientation.w = new_imu_msg.orientation.w;
        new_odom.pose.pose.orientation.x = new_imu_msg.orientation.x;
        new_odom.pose.pose.orientation.y = new_imu_msg.orientation.y;
        new_odom.pose.pose.orientation.z = new_imu_msg.orientation.z;
        new_odom.pose.pose.position.x = Odom_msg->pose.pose.position.x - UTMX0;
        new_odom.pose.pose.position.y = Odom_msg->pose.pose.position.y - UTMY0;
        new_odom.pose.pose.position.z = 1.2; // Odom_msg->pose.pose.position.z - alt0;
        pub_odom.publish(new_odom);

        zzz_driver_msgs::RigidBodyStateStamped state;
        state.header.frame_id = "map";
        state.header.stamp = Odom_msg->header.stamp;
        state.state.child_frame_id = "odom";
        state.state.pose.pose = new_odom.pose.pose;
        state.state.pose.pose.position.z = 0.0;
        state.state.twist.twist.linear = new_twist.twist.twist.linear;
        state.state.twist.twist.angular = new_imu_msg.angular_velocity;
        state.state.accel.accel.linear = new_imu_msg.linear_acceleration;
        pub_ego_pose.publish(state);

        Eigen::Vector3d xyz2(new_odom.pose.pose.position.x, new_odom.pose.pose.position.y, new_odom.pose.pose.position.z);
        Eigen::Quaterniond q_w_car;
        q_w_car.w() = new_odom.pose.pose.orientation.w;
        q_w_car.x() = new_odom.pose.pose.orientation.x;
        q_w_car.y() = new_odom.pose.pose.orientation.y;
        q_w_car.z() = new_odom.pose.pose.orientation.z;

        updateGTPath(Odom_msg->header, new_odom.pose.pose.position.x, new_odom.pose.pose.position.y, new_odom.pose.pose.position.z);
    }

    void pos_type_callback(const std_msgs::StringConstPtr &pos_type_msg)
    {
        pub_pos_type.publish(pos_type_msg);
    }

    void nav_status_callback(const std_msgs::StringConstPtr &Cvel_msg)
    {
        pub_nav_status.publish(Cvel_msg);
    }

    void gps_time_ref_callback(const sensor_msgs::TimeReferenceConstPtr &Time_ref)
    {
        pub_gps_time_ref.publish(Time_ref);
    }

    void updateGTPath(const std_msgs::Header header, double pos_x, double pos_y, double pos_z)
    {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = header.stamp; //ros::Time(iter->first);
        pose_stamped.header.frame_id = "utm";     //world
        pose_stamped.pose.position.x = pos_x;
        pose_stamped.pose.position.y = pos_y;
        pose_stamped.pose.position.z = pos_z;
        pose_stamped.pose.orientation.w = 1;
        pose_stamped.pose.orientation.x = 0;
        pose_stamped.pose.orientation.y = 0;
        pose_stamped.pose.orientation.z = 0;

        Groundtruth_path.poses.push_back(pose_stamped);
        Groundtruth_path.header = pose_stamped.header;
        pub_Groundtruth_path.publish(Groundtruth_path);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "global fusion oxford2");
    ROS_INFO("\033[1;32m---->\033[0m global_fusion oxford2 node Started.");

    global_fusion GF;
    ros::spin();
    return 0;
}