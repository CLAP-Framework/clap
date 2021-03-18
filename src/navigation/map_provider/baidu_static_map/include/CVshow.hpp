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

class CVShow {
public:
  CVShow() {
    grid_width_ = 100.0;
    grid_pixel_ = 500;
    grid_x_ = 2;
    grid_y_ = 2;
    width_ = grid_pixel_ * grid_x_;
    height_ = grid_pixel_ * grid_y_;
    stride_ = 3 * grid_width_ / (double)grid_pixel_;
    roi_width_ = 1000;
    roi_height_ = 1000;
    image_ = cv::Mat::zeros(cv::Size(width_, height_), CV_8UC3);

	  cv::namedWindow("Display map", CV_WINDOW_AUTOSIZE);
  }

  ~CVShow() {}

  bool isValid (cv::Point2i& pt) {
    return ( pt.x >= 0 && pt.x < width_ && pt.y >= 0 && pt.y < height_ );
  }

  cv::Point2i mapToImage (apollo::common::PointENU& center, apollo::common::PointENU& pt) {
    cv::Point2i ipt;
    ipt.x = static_cast<int>(std::floor((pt.x() - center.x() + 3*grid_width_) / stride_));
    ipt.y = height_ - 1 - static_cast<int>(std::floor((pt.y() - center.y() + 1*grid_width_) / stride_));
    return ipt;
  }

  void drawPoint(apollo::common::PointENU& center, 
      std::vector<apollo::common::PointENU>& geometry, 
      int radius, const cv::Scalar& color, 
      int thickness = 1, int lineType = cv::LineTypes::LINE_8, int shift = 0) {
    for (int j=0; j<geometry.size(); j++) {
      cv::Point2i ipt = mapToImage(center, geometry[j]);
      if (isValid(ipt)) {
        circle(image_, ipt, radius, color, thickness, lineType, shift);
      }                
    }
  }

  void drawLine(apollo::common::PointENU& center,
      std::vector<apollo::common::PointENU>& geometry, const cv::Scalar& color, 
      int thickness = 1, int lineType = cv::LineTypes::LINE_8, int shift = 0) {
    std::vector<cv::Point2i> ipt; 
    for (int j=0; j<geometry.size(); j++) {
      cv::Point2i ipt_tmp = mapToImage(center, geometry[j]);
      if (isValid(ipt_tmp)) {
        ipt.emplace_back(ipt_tmp);
      }   
    }
    if (ipt.size() > 1) {
      for (int j=0; j<(ipt.size()-1); j++) {
        line(image_, ipt[j], ipt[j+1], color, thickness, lineType, shift);   
      }
    }
  }

  void drawPoly(apollo::common::PointENU& center,
      std::vector<apollo::common::PointENU>& geometry, const cv::Scalar& color, 
      int lineType = cv::LineTypes::LINE_8, int shift = 0) {
    std::vector<cv::Point2i> ipt; 
    for (int j=0; j<geometry.size(); j++) {
      cv::Point2i ipt_tmp = mapToImage(center, geometry[j]);
      if (isValid(ipt_tmp)) {
        ipt.emplace_back(ipt_tmp);
      }   
    }
    cv::fillConvexPoly(image_, ipt.data(), ipt.size(), color, lineType, shift);
  }

  void show() {
    // cv::Mat img;
    // cv::resize(image_, img, cv::Size(width_/16, height_/16));
    // s_img, None, fx=0.1, fy=0.1, interpolation=cv2.INTER_AREA)
    cv::imshow("Display map", image_);
    cv::waitKey(100);
    image_ = cv::Mat::zeros(cv::Size(width_, height_), CV_8UC3);
  }

  // void show(GPS_Coord& gps) {
  //   cv::Point2i ipt = gpsToImage(gps);
  //   if (isValid(ipt)) {
  //     cv::Mat roi;
  //     // double angle = -gps.heading;
  //     // cv::Point2f center(image_merged.cols / 2, image_merged.rows / 2);
  //     // cv::Mat rot = cv::getRotationMatrix2D(center, angle, 1);
  //     // cv::warpAffine(image_merged, image, rot, image_merged.size());
  //     cv::Rect2i rect(std::max(0, (ipt.x - roi_width_>>1)), 
  //         std::max(0, (ipt.y - roi_height_>>1)), 
  //         std::min((ipt.x + roi_width_>>1), width_),
  //         std::min((ipt.y + roi_height_>>1), height_) );
  //     image_(rect).copyTo(roi); 
  //     cv::imshow("Display map", roi);
  //     cv::waitKey(0);
  //   }  
  // }

  void save() {
    std::string path = "../map.jpg";
    cv::imwrite(path, image_);    
  }

private:
  cv::Mat image_;

  int width_;
  int height_;
  double stride_;
  int roi_width_;
  int roi_height_;
  double grid_width_;
  int grid_pixel_;
  int grid_x_;
  int grid_y_;
};

