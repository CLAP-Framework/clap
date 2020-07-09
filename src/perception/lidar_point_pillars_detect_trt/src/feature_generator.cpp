#include "feature_generator.h"
#include <memory.h>
#include <cmath>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

bool FeatureGenerator::init(
    int feature_width, int feature_height,
    int max_pillar_num, int max_point_num, int feature_channel,\
    float min_vertical, float max_vertical, \
    float min_horizontal, float max_horizontal,\
    float min_height, float max_height) { 
    
    feature_width_ = feature_width;
    feature_height_ = feature_height;
    max_pillar_num_ = max_pillar_num;
    max_point_num_ = max_point_num;
    feature_channel_ = feature_channel;
    
    min_vertical_ = min_vertical;
    max_vertical_ = max_vertical;
    min_horizontal_ = min_horizontal; 
    max_horizontal_ = max_horizontal;
    min_height_ = min_height;
    max_height_ = max_height;  
    return true;
    // int pillar_num_;
    // std::vector<cv::Point> *coors_ = coors;
    // coors_ -> resize(max_pillar_num_);
    // std::vector<float> *voxels_ = voxels;
    // voxels_ -> resize(feature_channel_*max_pillar_num_*max_point_num_); 

}

void FeatureGenerator::generate(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_ptr,
                                float *coors, int &pillar_num, float *voxels) {
    auto& points = pc_ptr->points;
    float voxel_size_x = (max_vertical_ - min_vertical_) / feature_height_;
    float voxel_size_y = (max_horizontal_ - min_horizontal_) / feature_width_;
    cv::Mat coor_to_voxelidex = cv::Mat(feature_height_, feature_width_, CV_32SC1, \
                                        cv::Scalar(-1));
    int *num_points_per_voxel = new int[max_pillar_num_]();
    for (int32_t i = 0; i < points.size(); ++i) {
        points[i].z += 1.2;
        if (points[i].x >= min_vertical_ && points[i].x < max_vertical_ &&
            points[i].y >= min_horizontal_ && points[i].y < max_horizontal_ &&
            points[i].z >= min_height_ && points[i].z <= max_height_) {
            // 计算每个格子中心点
            int x_temp = floor((points[i].x - min_vertical_) / voxel_size_x); 
            int y_temp = floor((points[i].y - min_horizontal_) / voxel_size_y);
            int voxelidx = coor_to_voxelidex.at<int>(x_temp, y_temp);
            if (voxelidx == -1) {
                voxelidx = pillar_num;
                if (pillar_num < max_pillar_num_) {
                    pillar_num += 1;
                    coor_to_voxelidex.at<int>(x_temp, y_temp) = voxelidx;
                    coors[voxelidx * 2] = x_temp * 1.0;
                    coors[voxelidx * 2 + 1] = y_temp * 1.0;
                    int points_num = num_points_per_voxel[voxelidx];
                    if (points_num < max_point_num_) {
                        float center_x, center_y, voxel_x, voxel_y, voxel_z;
                        center_x = (x_temp + 0.5) * voxel_size_x + min_vertical_;
                        center_y = (y_temp + 0.5) * voxel_size_y + min_horizontal_;
                        voxel_x = (points[i].x - center_x) / voxel_size_x * 2;
                        voxel_y = (points[i].y - center_y) / voxel_size_y * 2;
                        voxel_z = points[i].z / 2.73; //max(abs(zmin),abs(zmax))
                        voxels[0 * max_point_num_ * max_pillar_num_ + \
                            (voxelidx * max_point_num_ + points_num)] = voxel_x;
                        voxels[1 * max_point_num_ * max_pillar_num_ + \
                            (voxelidx * max_point_num_ + points_num)] = voxel_y;
                        voxels[2 * max_point_num_ * max_pillar_num_ + \
                            (voxelidx * max_point_num_ + points_num)] = voxel_z;
                        num_points_per_voxel[voxelidx] += 1;
                    }
                } else {}
            } else {
                int points_num = num_points_per_voxel[voxelidx];
                // 保存x, y, z
                if (points_num < max_point_num_) {
                    float center_x, center_y, voxel_x, voxel_y, voxel_z;
                    center_x = (x_temp + 0.5) * voxel_size_x + min_vertical_;
                    center_y = (y_temp + 0.5) * voxel_size_y + min_horizontal_;
                    voxel_x = (points[i].x - center_x) / voxel_size_x * 2;
                    voxel_y = (points[i].y - center_y) / voxel_size_y * 2;
                    voxel_z = points[i].z / 2.73; //max(abs(zmin),abs(zmax))
                    voxels[0 * max_point_num_ * max_pillar_num_ + \
                        (voxelidx * max_point_num_ + points_num)] = voxel_x;
                    voxels[1 * max_point_num_ * max_pillar_num_ + \
                        (voxelidx * max_point_num_ + points_num)] = voxel_y;
                    voxels[2 * max_point_num_ * max_pillar_num_ + \
                        (voxelidx * max_point_num_ + points_num)] = voxel_z;
                    num_points_per_voxel[voxelidx] += 1;
                }
            }
        }
    }
    for (int voxel_idx = 0; voxel_idx < max_pillar_num_; ++voxel_idx) {
        if (num_points_per_voxel[voxel_idx] < max_point_num_) {
            for (int points_idx = num_points_per_voxel[voxel_idx]; \
                    points_idx < max_point_num_; ++points_idx) {
                voxels[0 * max_point_num_ * max_pillar_num_ + (voxel_idx * max_point_num_ + points_idx)] =\
                voxels[0 * max_point_num_ * max_pillar_num_ + (voxel_idx * max_point_num_ + 0)];
                voxels[1 * max_point_num_ * max_pillar_num_ + (voxel_idx * max_point_num_ + points_idx)] =\
                voxels[1 * max_point_num_ * max_pillar_num_ + (voxel_idx * max_point_num_ + 0)];
                voxels[2 * max_point_num_ * max_pillar_num_ + (voxel_idx * max_point_num_ + points_idx)] =\
                voxels[2 * max_point_num_ * max_pillar_num_ + (voxel_idx * max_point_num_ + 0)];
            }
        }
    }
    delete []num_points_per_voxel;
}