/**
* global_fusion_m2_.cpp
* @author jiangxx
* @description 
* @created 2020-05-13T17:27:27.559Z+08:00
* @copyright None 
* None
* @last-modified 2020-05-19T14:09:10.746Z+08:00  car Coordinate system: x_forword  y_left  z_up
*/

#include "ros/ros.h"
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/TimeReference.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>

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

#include "novatel_gnss_msgs/RawImu.h"
#include "novatel_gnss_msgs/InsPva.h"
#include "novatel_gnss_msgs/BestPosb.h"
#include "novatel_gnss_msgs/BestVelb.h"
#include "novatel_gnss_msgs/Headingb.h"

using namespace std;
using namespace novatel_gnss_msgs;

#define gyro_scale_factor (0.1/(3600*256.0))
#define acc_scale_factor (0.05/pow(2,15))


class global_fusion_m2
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

    //path
    nav_msgs::Path Groundtruth_path;

    //mutex
    std::mutex imu_mutex;
    std::mutex gps_mutex;
    std::mutex vel_mutex;
    std::mutex odom_mutex;

    // in topics name
    std::string gnss_inspva_topic;
    std::string gnss_rawimu_topic;
    std::string gnss_bestposb_topic;
    std::string gnss_bestvelb_topic;
    std::string gnss_headingb_topic;

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
    sensor_msgs::NavSatFix new_fix;
    geometry_msgs::TwistWithCovarianceStamped new_vel;
    geometry_msgs::TwistStamped new_twist;
    sensor_msgs::Imu new_imu;
    nav_msgs::Odometry new_odom;

    novatel_gnss_msgs::InsPva avp;

    std::mutex RAWimu_mutex;

    novatel_gnss_msgs::RawImu RawImu;

    boost::optional<Eigen::Vector3d> zero_utm;

public:
    global_fusion_m2()
    {
        // utm0_x = 0;
        // utm0_y = 0;
        lat0 = 0;
        lon0 = 0;
        alt0 = 0;
        debug = false;

        //in
        n.getParam("gnss_inspva_topic", gnss_inspva_topic);
        n.getParam("gnss_rawimu_topic", gnss_rawimu_topic);
        n.getParam("gnss_bestposb_topic", gnss_bestposb_topic);
        n.getParam("gnss_bestvelb_topic", gnss_bestvelb_topic);
        n.getParam("gnss_headingb_topic", gnss_headingb_topic);

        // n.getParam("utm0_x", utm0_x);
        // n.getParam("utm0_y", utm0_y);

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
    };
    ~global_fusion_m2() {}
    /**
     * 
    **/
    void NodeSetup()
    {
        //subscribe
        sub_fix = n.subscribe(gnss_inspva_topic, 100, &global_fusion_m2::inspva_callback, this);
        sub_imu = n.subscribe(gnss_rawimu_topic, 100, &global_fusion_m2::rawimu_callback, this);
        sub_vel = n.subscribe(gnss_bestposb_topic, 100, &global_fusion_m2::bestposb_callback, this);
        sub_odom = n.subscribe(gnss_bestvelb_topic, 100, &global_fusion_m2::bestvelb_callback, this);
        sub_pos_type = n.subscribe(gnss_headingb_topic, 100, &global_fusion_m2::headingb_callback, this);

        //Publisher
        pub_fix = n.advertise<sensor_msgs::NavSatFix>(gps_fix_topic, 100);
        pub_vel = n.advertise<geometry_msgs::TwistWithCovarianceStamped>(gps_vel_topic, 100);
        pub_imu = n.advertise<sensor_msgs::Imu>(gps_imu_topic, 100);
        pub_odom = n.advertise<nav_msgs::Odometry>(gps_odom_topic, 100);
        pub_pos_type = n.advertise<std_msgs::String>(gps_pos_type_topic, 100);
        pub_nav_status = n.advertise<std_msgs::String>(gps_nav_status_topic, 100);
        pub_gps_time_ref = n.advertise<sensor_msgs::TimeReference>(gps_time_ref_topic, 100);

        pub_car = n.advertise<visualization_msgs::MarkerArray>("car_model", 1000);
        pub_Groundtruth_path = n.advertise<nav_msgs::Path>("groundtruth_path", 100);
    }
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
    void rawimu_callback(const novatel_gnss_msgs::RawImu &RawImu_msg)
    {
        RAWimu_mutex.lock();

        RawImu = RawImu_msg;
        RAWimu_mutex.unlock();
    }

    void inspva_callback(const novatel_gnss_msgs::InsPva &InsPva_msg)
    {
        double W = 0, X = 0, Y = 0, Z = 0;
        double roll, pitch, yaw;
        roll = InsPva_msg.roll * M_PI / 180;
        pitch = InsPva_msg.pitch * M_PI / 180;
        yaw = InsPva_msg.azimuth * M_PI / 180;

        if (debug)
            std::cout << "imu 欧拉角roll：" << roll * 180 / M_PI << ",pitch:" << pitch * 180 / M_PI << ",yaw:" << yaw * 180 / M_PI << std::endl; // Q = q;

        //坐标系调整
        //坐标系从RFU(右前上)=>FLU(前左上), pitch需要加-才符合逆时针为正,roll不变,
        //yaw需要先从北向0=>东向0定义 yaw = 90-yaw,再约束到0~360度,
        pitch = -1 * pitch; //
        yaw = M_PI_2 - yaw; //转90度
        if (yaw < 0)
            yaw = yaw + 2 * M_PI; // 保证0~360范围
        if (yaw_n2w)
        {
            std::cout << "yaw_n2w" << std::endl;
            yaw = 2 * M_PI - yaw; ////yaw 逆时针 0-360度  星网yaw方向和Z敏感轴方向相反
        }

        if (debug)
            std::cout << "================>after 欧拉角roll：" << roll * 180 / M_PI << ",pitch:" << pitch * 180 / M_PI << ",yaw:" << yaw * 180 / M_PI << std::endl;
        att2q(roll, pitch, yaw, &W, &X, &Y, &Z);

        //gps/fix
        new_fix.header = InsPva_msg.header;
        new_fix.header.frame_id = "wgs84";
        new_fix.longitude = InsPva_msg.longitude;
        new_fix.latitude = InsPva_msg.latitude;
        new_fix.altitude = InsPva_msg.height;
        pub_fix.publish(new_fix); //fix

        ///For vel
        new_vel.header = InsPva_msg.header;
        new_vel.header.frame_id = "enu";
        new_vel.twist.twist.linear.x = InsPva_msg.east_velocity;
        new_vel.twist.twist.linear.y = InsPva_msg.north_velocity;
        new_vel.twist.twist.linear.z = InsPva_msg.up_velocity;
        pub_vel.publish(new_vel);

        //for new_imu
        //坐标系调换,imu需要x->-1*y_    y-->x_
	//由于y的neg存在,前面需要*-1  
        RAWimu_mutex.lock();
        new_imu.header = RawImu.header;
        new_imu.header.frame_id = "imu";
        new_imu.linear_acceleration.x = -1*RawImu.y_velocity_change_neg*acc_scale_factor*100;
        new_imu.linear_acceleration.y = RawImu.x_velocity_change*acc_scale_factor*100*(-1);
        new_imu.linear_acceleration.z = RawImu.z_velocity_change*acc_scale_factor*100;
        new_imu.angular_velocity.x = -1*RawImu.y_angle_change_neg*gyro_scale_factor*100;
        new_imu.angular_velocity.y = RawImu.x_angle_change*gyro_scale_factor*100*(-1);
        new_imu.angular_velocity.z = RawImu.z_angle_change*gyro_scale_factor*100;
        new_imu.orientation.w = W;
        new_imu.orientation.x = X;
        new_imu.orientation.y = Y;
        new_imu.orientation.z = Z;
        RAWimu_mutex.unlock();
        pub_imu.publish(new_imu);

        // //for new_twist
        // RAWimu_mutex.lock();
        // new_twist.header = RawImu.header;
        // new_twist.twist.linear.x = RawImu.x_velocity_change;
        // new_twist.twist.linear.y = RawImu.y_velocity_change_neg;
        // new_twist.twist.linear.z = RawImu.z_velocity_change;

        // new_twist.twist.angular.x = RawImu.x_angle_change;
        // new_twist.twist.angular.y = RawImu.y_angle_change_neg;
        // new_twist.twist.angular.z = RawImu.z_angle_change;
        // RAWimu_mutex.unlock();

        ///For odom

        /* code */
        double UTMX0 = utm0_x;
        double UTMY0 = utm0_y;
        std::string utm0_zone;
        gps_common::LLtoUTM(lat0, lon0, UTMY0, UTMX0, utm0_zone);

        double utm_x;
        double utm_y;
        std::string utm_zone;
        gps_common::LLtoUTM(InsPva_msg.latitude, InsPva_msg.longitude, utm_y, utm_x, utm_zone);

        new_odom.header = InsPva_msg.header;
        new_odom.header.frame_id = "utm";
        new_odom.pose.pose.orientation.w = W;
        new_odom.pose.pose.orientation.x = X;
        new_odom.pose.pose.orientation.y = Y;
        new_odom.pose.pose.orientation.z = Z;
        new_odom.pose.pose.position.x = utm_x - UTMX0;
        new_odom.pose.pose.position.y = utm_y - UTMY0;
        new_odom.pose.pose.position.z = InsPva_msg.height - alt0; //33.0999259949;
        pub_odom.publish(new_odom);                               //odom
    }

    void bestposb_callback(const novatel_gnss_msgs::BestPosb &BestPosb_msg)
    {
    }

    void bestvelb_callback(const novatel_gnss_msgs::BestVelb &BestVelb_msg)
    {
    }
    void headingb_callback(const novatel_gnss_msgs::Headingb &Headingb_msg)
    {
    }

    void updateGTPath(const std_msgs::Header header, double pos_x, double pos_y, double pos_z)
    {
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "global fusion m2");
    ROS_INFO("\033[1;32m---->\033[0m global_fusion m2 node Started.");

    global_fusion_m2 GF;
    ros::spin();
    return 0;
}
