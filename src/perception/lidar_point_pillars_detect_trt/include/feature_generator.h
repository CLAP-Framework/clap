#ifndef FEATURE_GENERATOR_H
#define FEATURE_GENERATOR_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <opencv2/opencv.hpp>
#include <vector>

class FeatureGenerator {
public:
  FeatureGenerator() {}
  ~FeatureGenerator() {}

  // bool init(std::vector<float>* out_blob);
  bool init(int feature_width, int feature_height,\
            int max_pillar_num, int max_point_num, int feature_channel,\
            float min_vertical, float max_vertical, \
            float min_horizontal, float max_horizontal,\
            float min_height, float max_height);
  void generate(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_ptr,\
                float *coors, int &pillar_num, float *voxels);

private:
  int feature_width_ = 320;
  int feature_height_ = 640;
  int max_pillar_num_ = 20000;
  int max_point_num_ = 32;
  int feature_channel_ = 3;
  
  float min_vertical_ = -50.f;
  float max_vertical_ = 50.f;
  float min_horizontal_ = -25.f; 
  float max_horizontal_ = 25.f;
  float min_height_ = -0.5f;
  float max_height_ = 3.5f;

  // raw feature data
  // int pillar_num_;
  // std::vector<cv::Point> *coors_;
  // std::vector<float> *voxels_;
  
};

#endif //FEATURE_GENERATOR_H
