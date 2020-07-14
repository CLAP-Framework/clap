#include "postprocess.h"
#include <exception>
#include <fstream>

bool PostProcess::init(float grid_width, float nms_threshold, 
    float confidence_car_threshold, float confidence_ped_cyc_threshold,\
    int dims_car_channel, int dims_car_height, int dims_car_width,\ 
    int dims_ped_cyc_channel, int dims_ped_cyc_height, int dims_ped_cyc_width,\ 
    float cls0_anchor_x, float cls0_anchor_y, float cls0_anchor_z,\
    float cls1_anchor_x, float cls1_anchor_y, float cls1_anchor_z,\
    float cls2_anchor_x, float cls2_anchor_y, float cls2_anchor_z,
    float min_height, float max_height) {

    grid_width_ = grid_width;
    nms_threshold_ = nms_threshold;
    confidence_car_threshold_ = confidence_car_threshold;
    confidence_ped_cyc_threshold_ = confidence_ped_cyc_threshold;
    score_threshold_sig_car_ = -log(1.0 / confidence_car_threshold_ - 1.0);
    score_threshold_sig_ped_cyc_ = -log(1.0 / confidence_ped_cyc_threshold_ - 1.0);
    dims_car_channel_ = dims_car_channel;
    dims_car_height_ = dims_car_height;
    dims_car_width_ = dims_car_width;
    dims_ped_cyc_channel_ = dims_ped_cyc_channel;
    dims_ped_cyc_height_ = dims_ped_cyc_height;
    dims_ped_cyc_width_ = dims_ped_cyc_width;
    cls0_anchor_x_ = cls0_anchor_x;
    cls0_anchor_y_ = cls0_anchor_y;
    cls0_anchor_z_ = cls0_anchor_z;
    cls1_anchor_x_ = cls1_anchor_x;
    cls1_anchor_y_ = cls1_anchor_y;
    cls1_anchor_z_ = cls1_anchor_z;
    cls2_anchor_x_ = cls2_anchor_x;
    cls2_anchor_y_ = cls2_anchor_y;
    cls2_anchor_z_ = cls2_anchor_z;
    min_height_ = min_height;
    max_height_ = max_height;
    return true;
}

float PostProcess::Sigmoid(float p) {
    return 1.0 / (1.0 + exp(-p * 1.0));
}
 
float PostProcess::BboxIou(cv::RotatedRect &rect1, cv::RotatedRect &rect2) {
    // 待计算旋转矩形的面积
    float areaRect1 = rect1.size.width * rect1.size.height;
    float areaRect2 = rect2.size.width * rect2.size.height;

    std::vector<cv::Point2f> vertices;
    // 计算两个旋转矩形的交集，返回值 0,1,2分别表示没有，有，包含。
    int intersectionType;
    // opencv对于几乎重合的两个矩阵,无法正常处理,会抛出异常
    // OpenCV Error: Assertion failed(intersection.size() <= 8) in rotatedRectangleIntersection.
    try {
        intersectionType = cv::rotatedRectangleIntersection(rect1, rect2, vertices);
    } catch (cv::Exception& e) {
        const char* err_msg = e.what();
        std::cout << err_msg << std::endl;
        return 1.0;
    }
    if (intersectionType == 0)
        return 0.0;
    else if (intersectionType == 2){
        float maxArea = areaRect1 > areaRect2 ? areaRect1 : areaRect2;
        float minArea = areaRect1 < areaRect2 ? areaRect1 : areaRect2;
        return minArea / maxArea;
    }
    else{
        std::vector<cv::Point2f> order_pts;
        // 找到交集（交集的区域），对轮廓的各个点进行排序，顺时针计算凸包点
        cv::convexHull(cv::Mat(vertices), order_pts, true);
        // 计算点所包围的多边形面积,点按逆时针或顺时针排布
        double area = cv::contourArea(order_pts);
        float inner = (float) (area / (areaRect1 + areaRect2 - area + 0.0001));
        return inner;
    }
}

// { x, y, w, h, angle, conf, 
//   Car, Van_Truck, Pedestrian_Person_sitting, Cyclist }
// class ClassedBox{
// public: 
//     ClassedBox() {}
//     // ClassedBox(float* raw, float* points) : invalid_(false)  {
//     ClassedBox(float* raw) : invalid_(false)  {
//         memcpy((void *)(&(this->raw_)), (void *)(raw), sizeof(RawData));
//         // memcpy((void *)(&(this->corner_)), (void *)(points), 32);
//         confidence_ = raw[7];
//     }
//     ~ClassedBox() {}
//     ClassedBox& operator= (const ClassedBox& from) {
//         memcpy((void *)(&(this->raw_)), (void *)(&(from.raw_)), sizeof(RawData));
//         this->confidence_ = from.confidence_;
//         this->invalid_ = from.invalid_;
//         return *this;
//     }
//     bool operator< (const ClassedBox& from) {
//         return (this->confidence_ < from.confidence_);
//     }
//     bool operator> (const ClassedBox& from) {
//         return (this->confidence_ > from.confidence_);
//     }
//     bool operator== (const ClassedBox& from) {
//         return (this->confidence_ == from.confidence_);
//     }
// public:
//     RawData raw_;
//     float confidence_;
//     bool invalid_;
// };

void PostProcess::BBoxToList(int cls_num, \
                float *bbox_result, int bbox_size,\
                std::vector<std::list<ClassedBox> > &class_boxes_list) {
    float conf_thre_temp;
    if (cls_num == 1) {
        conf_thre_temp = confidence_car_threshold_;
    } else {
        conf_thre_temp = confidence_ped_cyc_threshold_;
    }

    for(auto bbox_id = 0; bbox_id < bbox_size; bbox_id++){
        int index_bbox;
        if (cls_num == 1) {
            index_bbox = bbox_id * 8;
        } else if (cls_num == 2) {
            index_bbox = bbox_id * 10;
        }
        if(bbox_result[index_bbox + 7] > conf_thre_temp){
            // select the most posable class
            int max_idx = 0;
            if (cls_num != 1)
                max_idx = bbox_result[index_bbox + 8] > bbox_result[index_bbox + 9] ? 0 : 1;
            // 根据置信度排序
            ClassedBox box(&(bbox_result[index_bbox + 0]));
            if (class_boxes_list[max_idx].empty()) {
                class_boxes_list[max_idx].push_back(box);
            } 
            else {
                auto iter = class_boxes_list[max_idx].begin();
                for ( ; iter != class_boxes_list[max_idx].end(); iter++ ) {
                    if (*iter < box) {
                        class_boxes_list[max_idx].insert( iter, box );
                        break;
                    }
                }
                if(iter == class_boxes_list[max_idx].end())
                    class_boxes_list[max_idx].push_back(box);
            }
        }
    }
}

void PostProcess::BBoxToList(int cls_num, \
                std::vector<std::vector<float> > &bbox_result, \
                std::vector<std::list<ClassedBox> > &class_boxes_list) {
    float conf_thre_temp;
    if (cls_num == 1) {
        conf_thre_temp = confidence_car_threshold_;
    } else {
        conf_thre_temp = confidence_ped_cyc_threshold_;
    }
    
    for(auto bbox_id = 0; bbox_id < bbox_result.size(); bbox_id++){
        if(bbox_result[bbox_id][7] > conf_thre_temp){
            // select the most posable class
            int max_idx = 0;
            if (cls_num != 1)
                max_idx = bbox_result[bbox_id][8] > bbox_result[bbox_id][9] ? 0 : 1;
            // 根据置信度排序
            ClassedBox box(&(bbox_result[bbox_id][0]));
            if (class_boxes_list[max_idx].empty()) {
                class_boxes_list[max_idx].push_back(box);
            } 
            else {
                auto iter = class_boxes_list[max_idx].begin();
                for ( ; iter != class_boxes_list[max_idx].end(); iter++ ) {
                    if (*iter < box) {
                        class_boxes_list[max_idx].insert( iter, box );
                        break;
                    }
                }
                if(iter == class_boxes_list[max_idx].end())
                    class_boxes_list[max_idx].push_back(box);
            }
        }
    }
}

void PostProcess::NonMaxSuppression(int cls_num,\
                       std::vector<std::list<ClassedBox> > &class_boxes_list,
                       std::vector<BoundingBox> &result) {
    for (int class_idx = 0; class_idx < cls_num; class_idx++) {
        if (class_boxes_list[class_idx].empty()) {
            continue;
        }
        
        // computer iou
        for ( auto iter_first = class_boxes_list[class_idx].begin(); 
                   iter_first != class_boxes_list[class_idx].end(); 
                   iter_first++) {
            if ( iter_first->invalid_ ) {
                continue;
            } 
            // used mark 
            iter_first->invalid_ = true;
            for ( auto iter_sec =  class_boxes_list[class_idx].begin();
                       iter_sec != class_boxes_list[class_idx].end();
                       iter_sec++ ) {
                if ( iter_sec->invalid_ ) {
                    continue;
                } 
                // calculate IOU 
                cv::RotatedRect rect1(cv::Point2f(iter_first->raw_.x, iter_first->raw_.y), 
                                      cv::Size2f(iter_first->raw_.w, iter_first->raw_.l),
                                      iter_first->raw_.yaw * 180.0 / M_PI + 90.0);
                cv::RotatedRect rect2(cv::Point2f(iter_sec->raw_.x, iter_sec->raw_.y),
                                      cv::Size2f(iter_sec->raw_.w, iter_sec->raw_.l),
                                      iter_sec->raw_.yaw * 180.0 / M_PI + 90.0);
                float iou_val = BboxIou(rect1, rect2);
                if ( iou_val > nms_threshold_) {
                    iter_sec->invalid_ = true;
                }
            }

            // output boxls
            BoundingBox temp_bbox;
            temp_bbox.x = iter_first->raw_.x;
            temp_bbox.y = iter_first->raw_.y;
            temp_bbox.z = iter_first->raw_.z;
            temp_bbox.h = iter_first->raw_.h;
            temp_bbox.w = iter_first->raw_.w;
            temp_bbox.l = iter_first->raw_.l;
            temp_bbox.angle = iter_first->raw_.yaw;
            temp_bbox.class_prob = iter_first->confidence_;
            if (cls_num == 1) {
                temp_bbox.class_idx = class_idx;
            } else {
                temp_bbox.class_idx = class_idx + 1;
            }
            result.push_back(temp_bbox);
        }
    }
}

void PostProcess::PointCloudDetect(float *data, int h, int w, int c, 
                      int cls_num, float *anchors, 
                      std::vector<std::vector<float> > &result) {
    float conf_thre_temp;
    if (cls_num == 1) {
        conf_thre_temp = score_threshold_sig_car_;
    } else {
        conf_thre_temp = score_threshold_sig_ped_cyc_;
    }
    float *gpu_data = data;
    // x,y,z,h,w,l,im,re,conf,cls1,cls2
    std::vector<float> vec_data(c / cls_num);
    for (int h_index = 0; h_index < h; ++h_index) {
        for (int w_index = 0; w_index < w; ++w_index) {
            for (int c_index = 0; c_index < cls_num; ++c_index) {
                //0. int8_t convert to int.
                for (int vec_idx = 0; vec_idx < c / cls_num; ++vec_idx) {
                    vec_data[vec_idx] = gpu_data[h_index * w * c + 
                                                 w_index * c +
                                                 c_index * (c / cls_num) + vec_idx];
                }
                //1. obj confidence计算
                if(vec_data[8] < conf_thre_temp)
                    continue;
                std::vector<float> bbox(c / cls_num - 1);
                bbox[7] = Sigmoid(vec_data[8]);
                //2. 计算bbox预测角度
                bbox[6] = atan2(vec_data[6], 
                                vec_data[7]);
                //3. x y z sigmoid激活
                bbox[0] = Sigmoid(vec_data[0]);
                bbox[1] = Sigmoid(vec_data[1]);
                bbox[2] = Sigmoid(vec_data[2]);
                //4. h w l 指数激活
                bbox[3] = exp(vec_data[3]);
                bbox[4] = exp(vec_data[4]);
                bbox[5] = exp(vec_data[5]);
                //5. anchor计算
                bbox[0] = (bbox[0] + h_index) * (grid_width_ / w);
                bbox[1] = (bbox[1] + w_index) * (grid_width_ / w);
                bbox[2] = (bbox[2] * (max_height_ - min_height_)) +\
                                        min_height_;
                bbox[3] = bbox[3] * 1.7;
                
                if (cls_num > 1) {
                    bbox[4] = bbox[4] * anchors[(c_index + 1) * 3 + 0];
                    bbox[5] = bbox[5] * anchors[(c_index + 1) * 3 + 1];
                    bbox[6] = bbox[6] + anchors[(c_index + 1) * 3 + 2];
                    for(int cls_idx = 0; cls_idx < cls_num; ++cls_idx){
                        bbox[8 + cls_idx] = vec_data[9 + cls_idx];
                    }
                } else {
                    bbox[4] = bbox[4] * anchors[c_index * 3 + 0];
                    bbox[5] = bbox[5] * anchors[c_index * 3 + 1];
                    bbox[6] = bbox[6] + anchors[c_index * 3 + 2];
                }
                
                result.push_back(bbox);
            }
        }
    }
}

void PostProcess::PointCloudPostTreat(const std::string &label, float *data,
                std::vector<BoundingBox> &result) {
    float anchors[9] = {cls0_anchor_x_, cls0_anchor_y_, cls0_anchor_z_, 
                        cls1_anchor_x_, cls1_anchor_y_, cls1_anchor_z_, 
                        cls2_anchor_x_, cls2_anchor_y_, cls2_anchor_z_};
    int cls_num;
    std::vector<std::vector<float> > bbox_result;
    if (label == "car") {
        cls_num = 1;
        PointCloudDetect(data, dims_car_height_, dims_car_width_, dims_car_channel_, 
                         cls_num, anchors, bbox_result);
    } else if (label == "ped_cyc") {
        cls_num = 2;
        PointCloudDetect(data, dims_ped_cyc_height_, dims_ped_cyc_width_, 
                         dims_ped_cyc_channel_, 
                         cls_num, anchors, bbox_result);
    }
    std::vector<std::list<ClassedBox> > class_boxes_list(cls_num);
    BBoxToList(cls_num, bbox_result, class_boxes_list);
    NonMaxSuppression(cls_num, class_boxes_list, result);
}