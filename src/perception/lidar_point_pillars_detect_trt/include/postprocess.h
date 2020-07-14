#ifndef POSTPROCESS_H
#define POSTPROCESS_H

#include <iostream>
#include <vector>
#include <list>
#include <mutex>
#include <fstream>
#include <opencv2/opencv.hpp>

/// define struct to store lader data information
struct BoundingBox{
  int class_idx;
  float x;
  float y;
  float z;
  float h;
  float w;
  float l;
  float angle;
  float class_prob;
};

struct RawData {
    float x;
    float y;
    float z;
    float h;
    float w;
    float l;
    float yaw;
};

class ClassedBox{
public: 
    ClassedBox() {}
    // ClassedBox(float* raw, float* points) : invalid_(false)  {
    ClassedBox(float* raw) : invalid_(false)  {
        memcpy((void *)(&(this->raw_)), (void *)(raw), sizeof(RawData));
        // memcpy((void *)(&(this->corner_)), (void *)(points), 32);
        confidence_ = raw[7];
    }
    ~ClassedBox() {}
    ClassedBox& operator= (const ClassedBox& from) {
        memcpy((void *)(&(this->raw_)), (void *)(&(from.raw_)), sizeof(RawData));
        this->confidence_ = from.confidence_;
        this->invalid_ = from.invalid_;
        return *this;
    }
    bool operator< (const ClassedBox& from) {
        return (this->confidence_ < from.confidence_);
    }
    bool operator> (const ClassedBox& from) {
        return (this->confidence_ > from.confidence_);
    }
    bool operator== (const ClassedBox& from) {
        return (this->confidence_ == from.confidence_);
    }
public:
    RawData raw_;
    float confidence_;
    bool invalid_;
};
class PostProcess
{
public:
    PostProcess() {}
    ~PostProcess() {}
    bool init(float grid_width, float nms_threshold, \
        float confidence_car_threshold, float confidence_ped_cyc_threshold,\
        int dims_car_channel, int dims_car_height, int dims_car_width,\ 
        int dims_ped_cyc_channel, int dims_ped_cyc_height, int dims_ped_cyc_width,\ 
        float cls0_anchor_x, float cls0_anchor_y, float cls0_anchor_z,\
        float cls1_anchor_x, float cls1_anchor_y, float cls1_anchor_z,\
        float cls2_anchor_x, float cls2_anchor_y, float cls2_anchor_z,\
        float min_height, float max_height);
    
    void PointCloudPostTreat(const std::string &label, float *data, \
                             std::vector<BoundingBox> &result);
    float Sigmoid(float p);
    float BboxIou(cv::RotatedRect &rect1, cv::RotatedRect &rect2);
    void BBoxToList(int cls_num, \
            float *bbox_result, int bbox_size,\
            std::vector<std::list<ClassedBox> > &class_boxes_list);
    void BBoxToList(int cls_num, \
            std::vector<std::vector<float> > &bbox_result,
            std::vector<std::list<ClassedBox> > &class_boxes_list);
    void NonMaxSuppression(int cls_num, \
            std::vector<std::list<ClassedBox> > &class_boxes_list,
            std::vector<BoundingBox> &result);
    void PointCloudDetect(float *data, int h, int w, int c, 
            int cls_num, float *anchors,
            std::vector<std::vector<float> > &result);

private:
    float grid_width_ = 320;
    float nms_threshold_ = 0.3;
    float confidence_car_threshold_ = 0.4;
    float confidence_ped_cyc_threshold_ = 0.3;
    float score_threshold_sig_car_;
    float score_threshold_sig_ped_cyc_;
    int dims_car_channel_ = 9;
    int dims_car_height_ = 320;
    int dims_car_width_ = 160;
    int dims_ped_cyc_channel_ = 22;
    int dims_ped_cyc_height_ = 640;
    int dims_ped_cyc_width_ = 320;
    float cls0_anchor_x_ = 10.24;
    float cls0_anchor_y_ = 24.96;
    float cls0_anchor_z_ = 0.0;
    float cls1_anchor_x_ = 10.24;
    float cls1_anchor_y_ = 24.96;
    float cls1_anchor_z_ = 0.0;
    float cls2_anchor_x_ = 10.24;
    float cls2_anchor_y_ = 24.96;
    float cls2_anchor_z_ = 0.0;
    float min_height_ = -0.5;
    float max_height_ = 3.5;
};


#endif //POSTPROCESS_H
