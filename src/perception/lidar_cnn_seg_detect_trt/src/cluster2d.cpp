/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "cluster2d.h"
#include "detected_object.h"
#include <Eigen/Dense>
#include "bounding_line.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PointIndices.h>
#include <iostream>
bool Cluster2D::init(int rows, int cols, float range) {
  rows_ = rows;
  cols_ = cols;
  grids_ = rows_ * cols_;
  range_ = range;
  scale_ = 0.5 * static_cast<float>(rows_) / range_;
  inv_res_x_ = 0.5 * static_cast<float>(cols_) / range_;
  inv_res_y_ = 0.5 * static_cast<float>(rows_) / range_;
  point2grid_.clear();
  // obstacles_.clear();
  id_img_.assign(grids_, -1);
  pc_ptr_.reset();
  valid_indices_in_pc_ = nullptr;
  return true;
}

void Cluster2D::traverse(Node *x)
{
    std::vector<Node *> p;
    p.clear();
    while (x->traversed == 0) {
      p.push_back(x);
      x->traversed = 2;
      x = x->center_node;
    }
    if (x->traversed == 2)
    {
        for (int i = static_cast<int>(p.size()) - 1; i >= 0 && p[i] != x; i--)
        {
            p[i]->is_center = true;
        }
        x->is_center = true;
    }
    for (size_t i = 0; i < p.size(); i++)
    {
        Node *y = p[i];
        y->traversed = 1;
        y->parent = x->parent;
    }
}

void Cluster2D::cluster(const float* category_pt_blob,
                        const float* instance_pt_blob,
                        const pcl::PointCloud<pcl::PointXYZI>::Ptr & pc_ptr,
                        const pcl::PointIndices & valid_indices,
                        float objectness_thresh, 
                        bool use_all_grids_for_clustering)
{
    const float *category_pt_data = category_pt_blob;
    const float *instance_pt_x_data = instance_pt_blob;
    const float *instance_pt_y_data =
            instance_pt_blob + grids_;

    pc_ptr_ = pc_ptr;
    std::vector<std::vector<Node>> nodes(rows_,
                                         std::vector<Node>(cols_, Node()));

    // map points into grids
    size_t tot_point_num = pc_ptr_->size();
    valid_indices_in_pc_ = &(valid_indices.indices);
    // TODO:
    // CHECK_LE(valid_indices_in_pc_->size(), tot_point_num);
    if (valid_indices_in_pc_->size() > tot_point_num) {
        std::cerr << "valid_indices_in_pc_->size() needs to be less or equal than tot_point_num" << std::endl;
        // return ;
    } 
    point2grid_.assign(valid_indices_in_pc_->size(), -1);

    for (size_t i = 0; i < valid_indices_in_pc_->size(); ++i)
    {
        int point_id = valid_indices_in_pc_->at(i);
        // CHECK_GE(point_id, 0);
        if (point_id < 0) {
            std::cerr << "point_id needs to be greater or equal to 0" << std::endl;
            // continue;
        } 
        // CHECK_LT(point_id, static_cast<int>(tot_point_num));
        if (point_id >= static_cast<int>(tot_point_num)) {
             std::cerr << "equires point_id >= static_cast<int>(tot_point_num)" << std::endl;
            // continue;
        } 

        const auto &point = pc_ptr_->points[point_id];
        // * the coordinates of x and y have been exchanged in feature generation
        // step,
        // so we swap them back here.
        int pos_x = F2I(point.y, range_, inv_res_x_);  // col
        int pos_y = F2I(point.x, range_, inv_res_y_);  // row
        if (IsValidRowCol(pos_y, pos_x))
        {
            // get grid index and count point number for corresponding node
            point2grid_[i] = RowCol2Grid(pos_y, pos_x);
            nodes[pos_y][pos_x].point_num++;
        }
    }

    // construct graph with center offset prediction and objectness
    for (int row = 0; row < rows_; ++row)
    {
        for (int col = 0; col < cols_; ++col)
        {
            int grid = RowCol2Grid(row, col);
            Node *node = &nodes[row][col];
            DisjointSetMakeSet(node);
            node->is_object =
                    (use_all_grids_for_clustering || nodes[row][col].point_num > 0) &&
                    (*(category_pt_data + grid) >= objectness_thresh);
            int center_row = std::round(row + instance_pt_x_data[grid] * scale_);
            int center_col = std::round(col + instance_pt_y_data[grid] * scale_);
            center_row = std::min(std::max(center_row, 0), rows_ - 1);
            center_col = std::min(std::max(center_col, 0), cols_ - 1);
            node->center_node = &nodes[center_row][center_col];
        }
    }

    // traverse nodes
    for (int row = 0; row < rows_; ++row)
    {
        for (int col = 0; col < cols_; ++col)
        {
            Node *node = &nodes[row][col];
            if (node->is_object && node->traversed == 0)
            {
                traverse(node);
            }
        }
    }
    for (int row = 0; row < rows_; ++row)
    {
        for (int col = 0; col < cols_; ++col)
        {
            Node *node = &nodes[row][col];
            if (!node->is_center)
            {
                continue;
            }
            for (int row2 = row - 1; row2 <= row + 1; ++row2)
            {
                for (int col2 = col - 1; col2 <= col + 1; ++col2)
                {
                    if ((row2 == row || col2 == col) && IsValidRowCol(row2, col2))
                    {
                        Node *node2 = &nodes[row2][col2];
                        if (node2->is_center)
                        {
                            DisjointSetUnion(node, node2);
                        }
                    }
                }
            }
        }
    }

    int count_obstacles = 0;
    obstacles_.clear();
    id_img_.assign(grids_, -1);
    for (int row = 0; row < rows_; ++row)
    {
        for (int col = 0; col < cols_; ++col)
        {
            Node *node = &nodes[row][col];
            if (!node->is_object)
            {
                continue;
            }
            Node *root = DisjointSetFind(node);
            if (root->obstacle_id < 0)
            {
                root->obstacle_id = count_obstacles++;
                // CHECK_EQ(static_cast<int>(obstacles_.size()), count_obstacles - 1);
                if (static_cast<int>(obstacles_.size()) != (count_obstacles - 1)) {
                    std::cerr << "requires static_cast<int>(obstacles_.size()) != (count_obstacles - 1)" << std::endl;
                    // continue;
                } 
                obstacles_.push_back(Obstacle());
            }
            int grid = RowCol2Grid(row, col);
            // CHECK_GE(root->obstacle_id, 0);
            if (root->obstacle_id < 0) {
                std::cerr << "requires root->obstacle_id < 0" << std::endl;
                // continue;
            } 
            id_img_[grid] = root->obstacle_id;
            obstacles_[root->obstacle_id].grids.push_back(grid);
        }
    }
    // CHECK_EQ(static_cast<size_t>(count_obstacles), obstacles_.size());
}

void Cluster2D::filter(const float* confidence_pt_blob,
                       const float* height_pt_blob)
{
    const float *confidence_pt_data = confidence_pt_blob;
    const float *height_pt_data = height_pt_blob;
    for (size_t obstacle_id = 0; obstacle_id < obstacles_.size();
         obstacle_id++)
    {
        Obstacle *obs = &obstacles_[obstacle_id];
        // CHECK_GT(obs->grids.size(), 0);
        if (obs->grids.size() <= 0) {
            std::cerr << "requires obs->grids.size() <= 0" << std::endl;
            // continue;
        } 
        double score = 0.0;
        double height = 0.0;
        for (int grid : obs->grids)
        {
            score += static_cast<double>(confidence_pt_data[grid]);
            height += static_cast<double>(height_pt_data[grid]);
        }
        obs->score = score / static_cast<double>(obs->grids.size());
        obs->height = height / static_cast<double>(obs->grids.size());
        obs->cloud_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>);
    }
}

void Cluster2D::classify(const float* classify_pt_blob)
{
    const float *classify_pt_data = classify_pt_blob;
    int num_classes = MAX_META_TYPE;
    // CHECK_EQ(num_classes, MAX_META_TYPE);
    for (size_t obs_id = 0; obs_id < obstacles_.size(); obs_id++)
    {
        Obstacle *obs = &obstacles_[obs_id];
        for (size_t grid_id = 0; grid_id < obs->grids.size(); grid_id++)
        {
            int grid = obs->grids[grid_id];
            for (int k = 0; k < num_classes; k++)
            {
                obs->meta_type_probs[k] += classify_pt_data[k * grids_ + grid];
            }
        }
        int meta_type_id = 0;
        for (int k = 0; k < num_classes; k++)
        {
            obs->meta_type_probs[k] /= obs->grids.size();
            if (obs->meta_type_probs[k] > obs->meta_type_probs[meta_type_id])
            {
                meta_type_id = k;
            }
        }
        obs->meta_type = static_cast<MetaType>(meta_type_id);
    }
}

void Cluster2D::heading(const float* heading_pt_blob) {
    const float *heading_pt_data = heading_pt_blob;
    int num_heading = MAX_HEADING_MAP;
    // CHECK_EQ(num_heading, MAX_HEADING_MAP);
    const float *heading_x_ptr = heading_pt_data;
    const float *heading_y_ptr = heading_pt_data + grids_;

    for (size_t obs_id = 0; obs_id < obstacles_.size(); obs_id++) {
        Obstacle *obs = &obstacles_[obs_id];
        float heading_x = 0.0;
        float heading_y = 0.0;

        for (size_t grid_id = 0; grid_id < obs->grids.size(); grid_id++) {
            heading_x += heading_x_ptr[obs->grids[grid_id]];
            heading_y += heading_y_ptr[obs->grids[grid_id]];
        }
        obs->yaw = std::atan2(heading_y, heading_x) * 0.5f + M_PI;
        // convert yaw angle to [-M_PI, M_PI]
        obs->yaw = std::atan2(1.0 * sin(obs->yaw), 1.0 * cos(obs->yaw));
    }
}

nova::Object
Cluster2D::obstacleToObject(const Obstacle& in_obstacle, const nova::Header& in_header)
{
    nova::Object resulting_object;
    pcl::PointCloud<pcl::PointXYZI> in_cluster = *in_obstacle.cloud_ptr;
    resulting_object.header = in_header;
    resulting_object.score = in_obstacle.score;
    resulting_object.label = in_obstacle.meta_type;
    resulting_object.yaw = in_obstacle.yaw;

    if ( in_cluster.points.empty() ) {
        return resulting_object;
    }
    float min_z = std::numeric_limits<float>::max();
    float max_z = -std::numeric_limits<float>::max();
    float length, width, height, pose_x, pose_y, pose_z;
    float positive_yaw = std::abs(std::abs(in_obstacle.yaw) - M_PI/2);
    if ( positive_yaw < 0.087222 || positive_yaw > (M_PI/2 - 0.087222) ) {
        float min_x = std::numeric_limits<float>::max();
        float max_x = -std::numeric_limits<float>::max();
        float min_y = std::numeric_limits<float>::max();
        float max_y = -std::numeric_limits<float>::max();
        for (auto pit = in_cluster.points.begin(); pit != in_cluster.points.end(); ++pit)
        {
            if (pit->x < min_x)
                min_x = pit->x;
            if (pit->y < min_y)
                min_y = pit->y;
            if (pit->z < min_z)
                min_z = pit->z;
            if (pit->x > max_x)
                max_x = pit->x;
            if (pit->y > max_y)
                max_y = pit->y;
            if (pit->z > max_z)
                max_z = pit->z;
        }
        if ( positive_yaw < 0.087222 ) {
            length = abs(max_y - min_y);
            width = abs(max_x - min_x);
        } else {
            length = abs(max_x - min_x);
            width = abs(max_y - min_y);
        }
        height = abs(max_z - min_z);
        pose_x = (max_x + min_x) / 2 ;
        pose_y = (max_y + min_y) / 2 ;
        pose_z = (max_z + min_z) / 2 ;

    } else {
        float x1 = in_cluster.points.at(0).x;
        float y1 = in_cluster.points.at(0).y;
        ele::BoundingLine left_line(in_obstacle.yaw, x1, y1, ele::BoundingLine::BoundingType::Left);
        ele::BoundingLine right_line(in_obstacle.yaw, x1, y1, ele::BoundingLine::BoundingType::Right);
        ele::BoundingLine top_line(in_obstacle.yaw, x1, y1, ele::BoundingLine::BoundingType::Top);
        ele::BoundingLine bottom_line(in_obstacle.yaw, x1, y1, ele::BoundingLine::BoundingType::Bottom);
        min_z = in_cluster.points.at(0).z;
        max_z = in_cluster.points.at(0).z;
        for (auto pit = in_cluster.points.begin()+1; pit != in_cluster.points.end(); ++pit) {
            if (pit->z > max_z)
                max_z = pit->z; 
            if (pit->z < min_z)
                min_z = pit->z;
            left_line.Update(pit->x, pit->y);
            right_line.Update(pit->x, pit->y);
            top_line.Update(pit->x, pit->y);
            bottom_line.Update(pit->x, pit->y);
        }
        length = top_line.GetDistance(bottom_line);
        width = left_line.GetDistance(right_line);
        height = abs(max_z - min_z);
        float lt_x, lt_y, rb_x, rb_y;
        left_line.GetCross(top_line, lt_x, lt_y);
        right_line.GetCross(bottom_line, rb_x, rb_y); 
        pose_x = (lt_x + rb_x) / 2;
        pose_y = (lt_y + rb_y) / 2;
        pose_z = (max_z + min_z) / 2 ;
    }

    resulting_object.pose.x() = pose_x;
    resulting_object.pose.y() = pose_y;
    resulting_object.pose.z() = pose_z;
    resulting_object.dims.x() = length;
    resulting_object.dims.y() = width;
    resulting_object.dims.z() = height;

    Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(in_obstacle.yaw, Eigen::Vector3d::UnitZ()));
    Eigen::Quaterniond quaternion;
    quaternion = yawAngle;
    
    resulting_object.orie.x() = quaternion.x();
    resulting_object.orie.y() = quaternion.y();
    resulting_object.orie.z() = quaternion.z();
    resulting_object.orie.w() = quaternion.w();
#if 0
    std::cout << std::to_string(resulting_object.header.seq) << " "
              << resulting_object.label << " "
              << std::to_string(resulting_object.dims.x()) << " "
              << std::to_string(resulting_object.dims.y()) << " "
              << std::to_string(resulting_object.dims.z()) << " "
              << std::to_string(resulting_object.pose.x()) << " "
              << std::to_string(resulting_object.pose.y()) << " "
              << std::to_string(resulting_object.pose.z()) << " "
              << std::to_string(resulting_object.orie.x()) << " "
              << std::to_string(resulting_object.orie.y()) << " "
              << std::to_string(resulting_object.orie.z()) << " "
              << std::to_string(resulting_object.orie.w()) << " "
              << std::to_string(resulting_object.yaw) << " "
              << std::to_string(resulting_object.score) << std::endl;
#endif
    return resulting_object;
}

void Cluster2D::getObjects(const float confidence_thresh,
                           const float height_thresh,
                           const int min_pts_num,
                           std::vector<nova::Object> & objects,
                           const nova::Header & in_header)
{
    // CHECK(valid_indices_in_pc_ != nullptr);
    if (valid_indices_in_pc_ == nullptr) {
        return ;
    }

    for (size_t i = 0; i < point2grid_.size(); ++i)
    {
        int grid = point2grid_[i];
        if (grid < 0)
        {
            continue;
        }

        // CHECK_GE(grid, 0);
        if (grid < 0) {
            continue;
        }
        // CHECK_LT(grid, grids_);
        if (grid >= grids_) {
            continue;
        }

        int obstacle_id = id_img_[grid];

        int point_id = valid_indices_in_pc_->at(i);
        // CHECK_GE(point_id, 0);
        if (point_id < 0) {
            continue;
        }
        // CHECK_LT(point_id, static_cast<int>(pc_ptr_->size()));
        if (point_id >= static_cast<int>(pc_ptr_->size())) {
            continue;
        }
        if (obstacle_id >= 0 &&
            obstacles_[obstacle_id].score >= confidence_thresh)
        {
            if (height_thresh < 0 ||
                pc_ptr_->points[point_id].z <=
                obstacles_[obstacle_id].height + height_thresh)
            {
                obstacles_[obstacle_id].cloud_ptr->push_back(pc_ptr_->points[point_id]);
            }
        }
    }

    for (size_t obstacle_id = 0; obstacle_id < obstacles_.size();
         obstacle_id++)
    {
        Obstacle *obs = &obstacles_[obstacle_id];
        if (static_cast<int>(obs->cloud_ptr->size()) < min_pts_num) {
            continue;
        }

        nova::Object out_obj = obstacleToObject(*obs, in_header);

        // if ( std::abs(out_obj.pose.x()) < 2.0 
        //         && std::abs(out_obj.pose.y()) < 1.0) {
        //     continue;
        // }

        objects.push_back(out_obj);
    }
}

ObjectType getObjectType(const MetaType meta_type_id)
{
    switch (meta_type_id)
    {
        case META_UNKNOWN:
            return UNKNOWN;
        case META_SMALLMOT:
            return VEHICLE;
        case META_BIGMOT:
            return VEHICLE;
        case META_NONMOT:
            return BICYCLE;
        case META_PEDESTRIAN:
            return PEDESTRIAN;
        default:
        {
            return UNKNOWN;
        }
    }
}
