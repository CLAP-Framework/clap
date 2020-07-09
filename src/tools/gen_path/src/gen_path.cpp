#include "ros/ros.h"
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
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
using namespace std;
class gen_path
{

private:
  ros::NodeHandle n;
  ros::Subscriber sub_GPS;
  ros::Subscriber sub_imu;

  boost::optional<Eigen::Vector3d> zero_utm;
  Eigen::Vector3d last_utm;

  std::string path_file;
  std::string gps_topic;

  std::mutex imu_mutex;

  Eigen::Quaterniond Q;

public:
  gen_path() : Q(1.0, 0.0, 0.0, 0.0)
  {

    n.getParam("path_file", path_file);
    n.getParam("gps_topic", gps_topic);

    cout << "gps_topic:" << gps_topic << endl;
    cout << "path_file:" << path_file << endl;
    NodeSetup();
    //threadProcess = std::thread(&GlobalOptNode::process, this);
  }

  ~gen_path()
  {
    //threadProcess.detach();
  }
  /**
    **/
  void NodeSetup()
  {
    sub_GPS = n.subscribe(gps_topic, 100, &gen_path::GPS_callback, this);

    sub_imu = n.subscribe("/imu/data", 100, &gen_path::IMU_callback, this);
  }

  void IMU_callback(const sensor_msgs::ImuPtr &imu_msg)
  {
    std::lock_guard<std::mutex> lock(imu_mutex);

    float w = imu_msg->orientation.w;
    float x = imu_msg->orientation.x;
    float y = imu_msg->orientation.y;
    float z = imu_msg->orientation.z;

#if 1
    float Roll = atan2((2 * (w*x + y*z)), (1 - 2 * (x*x + y*y)));
    float Pitch = asin(2 * (w*y - z*x));
    float Yaw = atan2((2 * (w*z + x*y)), (1 - 2 * (z*z + y*y)));

    Yaw = Yaw + 3.141592654/2;
    if (Yaw < 0) {
        Yaw = Yaw + 2*3.1415927;
    }
    //std::cout << "New yaw = " << Yaw/3.1415927*180 << std::endl;
    double cy = cos(Yaw * 0.5);
    double sy = sin(Yaw * 0.5);
    double cp = cos(Pitch * 0.5);
    double sp = sin(Pitch * 0.5);
    double cr = cos(Roll * 0.5);
    double sr = sin(Roll * 0.5); 

    w = cy * cp * cr + sy * sp * sr;
    x = cy * cp * sr - sy * sp * cr;
    y = sy * cp * sr + cy * sp * cr;
    z = sy * cp * cr - cy * sp * sr; 
#endif

    Eigen::Quaterniond q(w, x, y, z);
    Q = q;
  }
  /***
    *
    * */
  void GPS_callback(const sensor_msgs::NavSatFixConstPtr &GPS_msg)
  {
    geographic_msgs::GeoPointStampedPtr gps_msg(new geographic_msgs::GeoPointStamped());
    gps_msg->header = GPS_msg->header;
    gps_msg->position.latitude = GPS_msg->latitude;
    gps_msg->position.longitude = GPS_msg->longitude;
    gps_msg->position.altitude = GPS_msg->altitude;

    // convert (latitude, longitude, altitude) -> (easting, northing, altitude) in UTM coordinate
    geodesy::UTMPoint utm;
    geodesy::fromMsg(gps_msg->position, utm);
    Eigen::Vector3d xyz(utm.easting, utm.northing, utm.altitude);

    if (!zero_utm)
    {
      zero_utm = xyz;

      std::lock_guard<std::mutex> lock(imu_mutex);
      //write to txt
      std::ofstream foutC(path_file, ios::app);
      foutC.setf(ios::fixed, ios::floatfield);
      // foutC.precision(0);
      // foutC << aloam_t * 1e9 << " ";
      // foutC << gps_t << " ";
      foutC.precision(5);

      // foutC << GPS_msg->longitude << " "
      //       << GPS_msg->latitude << std::endl;
      foutC << GPS_msg->longitude << " "
              << GPS_msg->latitude << " "
              << Q.w() << " "
              << Q.x() << " "
              << Q.y() << " "
              << Q.z() << std::endl;

      foutC.close();
    }
    else
    {
      double movement = (xyz - last_utm).norm();
      cout << "movement" << movement << endl;
      if (movement > 0.2)
      {
        std::lock_guard<std::mutex> lock(imu_mutex);

        std::ofstream foutC(path_file, ios::app);
        foutC.setf(ios::fixed, ios::floatfield);
        // foutC.precision(0);
        // foutC << aloam_t * 1e9 << " ";
        // foutC << gps_t << " ";
        foutC.precision(9);
        // foutC << xyz[0] << " "
        //       << xyz[1] << std::endl;
        foutC << GPS_msg->longitude << " "
              << GPS_msg->latitude << " "
              << Q.w() << " "
              << Q.x() << " "
              << Q.y() << " "
              << Q.z() << std::endl;

        foutC.close();

        last_utm = xyz;
        cout << "write to file..." << endl;
      }
      else
        cout << "small trans" << endl;
    }

    // the first gps data position will be the origin of the map
    // if (!zero_utm)
    // {
    //   zero_utm = xyz;
    // }
    // xyz -= (*zero_utm);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gen_path");
  ROS_INFO("\033[1;32m---->\033[0m gen_path Started.");

  gen_path GP;
  ros::spin();
  return 0;
}
