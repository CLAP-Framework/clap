#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "zzz_navigation_msgs/Map.h"
#include "zzz_driver_msgs/RigidBodyStateStamped.h"
#include "modules/map/hdmap/hdmap.h"
#include <visualization_msgs/MarkerArray.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>

#include <cmath>
#include <map>
#include <vector>
#include <memory>
#include <iomanip> // for shared_ptr
#include <iostream>
#include <string>
#include <yaml-cpp/yaml.h>
#include <fstream>

class Utm {
public:
  Utm();
  apollo::common::PointENU fromLatLon(double latitude, double longitude, 
      int force_zone_number, char force_zone_letter);

private:
  bool inBounds(double x, double lower, double upper); 
  double radians(double deg); 
  int zone_number_to_central_longitude(int zone_number);

private:
  double K0;
  double E ;
  double E2;
  double E3;
  double E_P2 ;

  double SQRT_E ;
  double _E ;
  double _E2;
  double _E3;
  double _E4;
  double _E5;

  double M1 ;
  double M2 ;
  double M3 ;
  double M4 ;

  double P2 ;
  double P3 ;
  double P4 ;
  double P5 ;

  double R ;
  std::string ZONE_LETTERS ;
};

Utm::Utm() {
  K0 = 0.9996;
  E = 0.00669438;
  E2 = E * E;
  E3 = E2 * E;
  E_P2 = E / (1.0 - E);

  SQRT_E = sqrt(1 - E);
  _E = (1 - SQRT_E) / (1 + SQRT_E);
  _E2 = _E * _E;
  _E3 = _E2 * _E;
  _E4 = _E3 * _E;
  _E5 = _E4 * _E;

  M1 = (1 - E / 4 - 3 * E2 / 64 - 5 * E3 / 256);
  M2 = (3 * E / 8 + 3 * E2 / 32 + 45 * E3 / 1024);
  M3 = (15 * E2 / 256 + 45 * E3 / 1024);
  M4 = (35 * E3 / 3072);

  P2 = (3. / 2 * _E - 27. / 32 * _E3 + 269. / 512 * _E5);
  P3 = (21. / 16 * _E2 - 55. / 32 * _E4);
  P4 = (151. / 96 * _E3 - 417. / 128 * _E5);
  P5 = (1097. / 512 * _E4);

  R = 6378137;
  ZONE_LETTERS = "CDEFGHJKLMNPQRSTUVWXX";
}

apollo::common::PointENU Utm::fromLatLon(double latitude, double longitude, 
    int force_zone_number=50, char force_zone_letter='S') {
  if (!inBounds(latitude, -80.0, 84.0)) {
    std::cout << "latitude out of range (must be between 80 deg S and 84 deg N)" << std::endl;
  }
  if (!inBounds(longitude, -180.0, 180.0)) {
    std::cout << "longitude out of range (must be between 180 deg W and 180 deg E)" << std::endl;
  }

  double lat_rad = radians(latitude);
  double lat_sin = sin(lat_rad);
  double lat_cos = cos(lat_rad);

  double lat_tan = lat_sin / lat_cos;
  double lat_tan2 = lat_tan * lat_tan;
  double lat_tan4 = lat_tan2 * lat_tan2;

  int zone_number = force_zone_number;
  int zone_letter = force_zone_letter;

  double lon_rad = radians(longitude);
  double central_lon = zone_number_to_central_longitude(zone_number);
  double central_lon_rad = radians(central_lon);

  double n = R / sqrt(1 - E * lat_sin * lat_sin);
  double c = E_P2 * lat_cos * lat_cos;

  double a = lat_cos * (lon_rad - central_lon_rad);
  double a2 = a * a;
  double a3 = a2 * a;
  double a4 = a3 * a;
  double a5 = a4 * a;
  double a6 = a5 * a;

  double m = R * (M1 * lat_rad -
            M2 * sin(2 * lat_rad) +
            M3 * sin(4 * lat_rad) -
            M4 * sin(6 * lat_rad));
  apollo::common::PointENU out;
  out.set_x( K0 * n * (a + a3 / 6 * (1 - lat_tan2 + c) +
      a5 / 120 * (5 - 18 * lat_tan2 + lat_tan4 + 72 * c - 58 * E_P2)) + 500000 );
  out.set_y( K0 * (m + n * lat_tan * (a2 / 2 + a4 / 24 * (5 - lat_tan2 + 9 * c + 4 * c * c) +
      a6 / 720 * (61 - 58 * lat_tan2 + lat_tan4 + 600 * c - 330 * E_P2))) );
  return out;
}
bool Utm::inBounds(double x, double lower, double upper) {
  return (lower < x) && (x < upper);
}
double Utm::radians(double deg) {
  return (deg / 180.0 * M_PI);
}
int Utm::zone_number_to_central_longitude(int zone_number) {
  return ((zone_number - 1) * 6 - 180 + 3); 
}