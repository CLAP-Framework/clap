#include "postprocess_cuda.h"

template <typename scalar_t>
__device__ __forceinline__ scalar_t Sigmoid(scalar_t z) {
  return 1.0 / (1.0 + expf(-z * 1.0));
}

__global__ void detect_car_kernel(const float *nn_out, const int grid_x,
        const int feature_w, const int feature_c,
        const float sigmoid_conf_thre, 
        const float min_z, const float max_z,
        const float anchor_car_w, const float anchor_car_l,
        int *filter_count, float *filtered_box) {
    int x = threadIdx.x;
    int y = blockIdx.x;
    int idx = (y * feature_w + x) * feature_c;
    if (nn_out[idx + 8] < sigmoid_conf_thre)
        return;
    int counter = atomicAdd(filter_count, 1);
    float box_score = 1.0 / (1.0 + exp(-(nn_out[idx + 8]) * 1.0));
    //2. 计算bbox预测角度
    float box_angle = atan2(nn_out[idx + 6], nn_out[idx + 7]);
    //3. x y z sigmoid激活
    float box_x = Sigmoid(nn_out[idx + 0]);
    float box_y = Sigmoid(nn_out[idx + 1]);
    float box_z = Sigmoid(nn_out[idx + 2]);
    //4. h w l 指数激活
    float box_h = expf(nn_out[idx + 3]) * 1.7;
    float box_w = expf(nn_out[idx + 4]) * anchor_car_w;
    float box_l = expf(nn_out[idx + 5]) * anchor_car_l;
    //5. anchor计算
    box_x = (box_x + y) * (grid_x / feature_w);
    box_y = (box_y + x) * (grid_x / feature_w);
    box_z = (box_z * (max_z - min_z)) + min_z;

    filtered_box[counter * 8 + 0] = box_x;
    filtered_box[counter * 8 + 1] = box_y;
    filtered_box[counter * 8 + 2] = box_z;
    filtered_box[counter * 8 + 3] = box_h;
    filtered_box[counter * 8 + 4] = box_w;
    filtered_box[counter * 8 + 5] = box_l;
    filtered_box[counter * 8 + 6] = box_angle;
    filtered_box[counter * 8 + 7] = box_score;
}

__global__ void detect_ped_cyc_kernel(const float *nn_out, const int grid_x,
        const int feature_w, const int feature_c,
        const float sigmoid_conf_thre, 
        const float min_z, const float max_z,
        const float anchor_ped_w, const float anchor_ped_l,
        const float anchor_cyc_w, const float anchor_cyc_l,
        int *filter_count, float *filtered_box) {
    int x = threadIdx.x;
    int y = blockIdx.x;
    int idx = (y * feature_w + x) * feature_c;
    for (int i = 0; i < 2; ++i) {
        if (nn_out[idx + i*11 + 8] < sigmoid_conf_thre)
            continue;
        int counter = atomicAdd(filter_count, 1);
        float box_score = 1.0 / (1.0 + exp(-(nn_out[idx + i*11 + 8]) * 1.0));
        //2. 计算bbox预测角度
        float box_angle = atan2(nn_out[idx + i*11 + 6], nn_out[idx + i*11 + 7]);
        //3. x y z sigmoid激活
        float box_x = Sigmoid(nn_out[idx + i*11 + 0]);
        float box_y = Sigmoid(nn_out[idx + i*11 + 1]);
        float box_z = Sigmoid(nn_out[idx + i*11 + 2]);
        //4. h w l 指数激活
        float box_h = expf(nn_out[idx + i*11 + 3]) * 1.7;
        float box_w;
        float box_l;
        if (i == 0) {
            box_w = expf(nn_out[idx + i*11 + 4]) * anchor_ped_w;
            box_l = expf(nn_out[idx + i*11 + 5]) * anchor_ped_l;
        } else if (i == 1) {
            box_w = expf(nn_out[idx + i*11 + 4]) * anchor_cyc_w;
            box_l = expf(nn_out[idx + i*11 + 5]) * anchor_cyc_l;
        }
        
        //5. anchor计算
        box_x = (box_x + y) * (grid_x / feature_w);
        box_y = (box_y + x) * (grid_x / feature_w);
        box_z = (box_z * (max_z - min_z)) + min_z;

        filtered_box[counter * 10 + 0] = box_x;
        filtered_box[counter * 10 + 1] = box_y;
        filtered_box[counter * 10 + 2] = box_z;
        filtered_box[counter * 10 + 3] = box_h;
        filtered_box[counter * 10 + 4] = box_w;
        filtered_box[counter * 10 + 5] = box_l;
        filtered_box[counter * 10 + 6] = box_angle;
        filtered_box[counter * 10 + 7] = box_score;
        filtered_box[counter * 10 + 8] = nn_out[idx + i*11 + 9];
        filtered_box[counter * 10 + 9] = nn_out[idx + i*11 + 10];
    }
}

PostprocessCuda::PostprocessCuda(const float MIN_Z, const float MAX_Z,
    const int GRID_X, const int NUM_ANCHOR_X_CAR_INDS, 
    const int NUM_ANCHOR_Y_CAR_INDS, const int NUM_ANCHOR_C_CAR_INDS,
    const int NUM_ANCHOR_X_PED_CYC_INDS, const int NUM_ANCHOR_Y_PED_CYC_INDS,
    const int NUM_ANCHOR_C_PED_CYC_INDS,
    const float ANCHOR_CAR_W, const float ANCHOR_CAR_L, 
    const float ANCHOR_PED_W, const float ANCHOR_PED_L, 
    const float ANCHOR_CYC_W, const float ANCHOR_CYC_L,
    const float score_car_threshold, const float score_ped_cyc_threshold,
    const float nms_threshold,
    const int NUM_OUTPUT_BOX_CAR_FEATURE,
    const int NUM_OUTPUT_BOX_PED_CYC_FEATURE):
    MIN_Z_(MIN_Z),
    MAX_Z_(MAX_Z),
    GRID_X_(GRID_X),
    NUM_ANCHOR_X_CAR_INDS_(NUM_ANCHOR_X_CAR_INDS),
    NUM_ANCHOR_Y_CAR_INDS_(NUM_ANCHOR_Y_CAR_INDS),
    NUM_ANCHOR_C_CAR_INDS_(NUM_ANCHOR_C_CAR_INDS),
    NUM_ANCHOR_X_PED_CYC_INDS_(NUM_ANCHOR_X_PED_CYC_INDS),
    NUM_ANCHOR_Y_PED_CYC_INDS_(NUM_ANCHOR_Y_PED_CYC_INDS),
    NUM_ANCHOR_C_PED_CYC_INDS_(NUM_ANCHOR_C_PED_CYC_INDS),
    ANCHOR_CAR_W_(ANCHOR_CAR_W),
    ANCHOR_CAR_L_(ANCHOR_CAR_L),
    ANCHOR_PED_W_(ANCHOR_PED_W),
    ANCHOR_PED_L_(ANCHOR_PED_L),
    ANCHOR_CYC_W_(ANCHOR_CYC_W),
    ANCHOR_CYC_L_(ANCHOR_CYC_L),
    score_car_threshold_(score_car_threshold),
    score_ped_cyc_threshold_(score_ped_cyc_threshold),
    nms_threshold_(nms_threshold),
    NUM_OUTPUT_BOX_CAR_FEATURE_(NUM_OUTPUT_BOX_CAR_FEATURE),
    NUM_OUTPUT_BOX_PED_CYC_FEATURE_(NUM_OUTPUT_BOX_PED_CYC_FEATURE) {
        sigmoid_score_car_threshold_ = -log(1.0 / score_car_threshold_ - 1.0);
        sigmoid_score_ped_cyc_threshold_ = -log(1.0 / score_ped_cyc_threshold_ - 1.0);
        host_car_bbox_ = (float*)malloc(NUM_ANCHOR_X_CAR_INDS_ * NUM_ANCHOR_Y_CAR_INDS_ * 8 * sizeof(float));
        host_ped_cyc_bbox_ = (float*)malloc(NUM_ANCHOR_X_PED_CYC_INDS_ * NUM_ANCHOR_Y_PED_CYC_INDS_ * 10 * sizeof(float));
        nbytes_car_filter_count_ = NUM_ANCHOR_Y_CAR_INDS_ * NUM_ANCHOR_X_CAR_INDS_ * 8 * sizeof(float);
        nbytes_ped_cyc_filter_count_ = NUM_ANCHOR_X_PED_CYC_INDS_ * NUM_ANCHOR_Y_PED_CYC_INDS_ * 10 * sizeof(float);
        GPU_CHECK(cudaMalloc((void**)&dev_car_filter_count_, sizeof(int)));
        GPU_CHECK(cudaMalloc((void**)&dev_car_filter_box_, nbytes_car_filter_count_));
        GPU_CHECK(cudaMalloc((void**)&dev_ped_cyc_filter_count_, sizeof(int)));
        GPU_CHECK(cudaMalloc((void**)&dev_ped_cyc_filter_box_, nbytes_ped_cyc_filter_count_));
        bounding_box_.reset(new PostProcess());
        bounding_box_ -> init(GRID_X_, nms_threshold_, 
            score_car_threshold_, score_ped_cyc_threshold_,\
            NUM_ANCHOR_C_CAR_INDS_, NUM_ANCHOR_Y_CAR_INDS_, NUM_ANCHOR_X_CAR_INDS_,\
            NUM_ANCHOR_C_PED_CYC_INDS_, NUM_ANCHOR_Y_PED_CYC_INDS_, NUM_ANCHOR_X_PED_CYC_INDS_,\
            ANCHOR_CAR_W_, ANCHOR_CAR_L_, 0.0f,\
            ANCHOR_PED_W_, ANCHOR_PED_L_, 0.0f,\
            ANCHOR_CYC_W_, ANCHOR_CYC_L_, 0.0f,\
            MIN_Z_, MAX_Z_);
}

PostprocessCuda::~PostprocessCuda() {
    free(host_car_bbox_);
    free(host_ped_cyc_bbox_);
    GPU_CHECK(cudaFree(dev_car_filter_count_));
    GPU_CHECK(cudaFree(dev_car_filter_box_));
    GPU_CHECK(cudaFree(dev_ped_cyc_filter_count_));
    GPU_CHECK(cudaFree(dev_ped_cyc_filter_box_));
}

void PostprocessCuda::doPostprocessCuda(const std::string &label, 
        const float *nn_out, 
        // float *filtered_box, int *filter_count,
        const int NUM_THREADS,
        std::vector<BoundingBox> &result) {
    if (label == "car") {
        GPU_CHECK(cudaMemset(dev_car_filter_count_, 0, sizeof(int)));
        GPU_CHECK(cudaMemset(dev_car_filter_box_, 0, nbytes_car_filter_count_));
        std::vector<std::list<ClassedBox> > class_boxes_list(1);
        detect_car_kernel<<<NUM_ANCHOR_Y_CAR_INDS_, NUM_ANCHOR_X_CAR_INDS_>>>
                (nn_out, GRID_X_, 
                NUM_ANCHOR_X_CAR_INDS_, NUM_ANCHOR_C_CAR_INDS_,
                sigmoid_score_car_threshold_, MIN_Z_, MAX_Z_,
                ANCHOR_CAR_W_, ANCHOR_CAR_L_,
                dev_car_filter_count_, dev_car_filter_box_);
        int host_filter_car_count[1];
        GPU_CHECK(cudaMemcpy(host_filter_car_count, dev_car_filter_count_,
                             sizeof(int), cudaMemcpyDeviceToHost));
        GPU_CHECK(cudaMemcpy(host_car_bbox_, dev_car_filter_box_, 
                host_filter_car_count[0] * 8 * sizeof(float), cudaMemcpyDeviceToHost));
        bounding_box_ -> BBoxToList(1, host_car_bbox_, 
                                    host_filter_car_count[0], class_boxes_list);
        bounding_box_ -> NonMaxSuppression(1, class_boxes_list, result);

    } else if (label == "ped_cyc") {
        GPU_CHECK(cudaMemset(dev_ped_cyc_filter_count_, 0, sizeof(int)));
        GPU_CHECK(cudaMemset(dev_ped_cyc_filter_box_, 0, nbytes_ped_cyc_filter_count_));
        std::vector<std::list<ClassedBox> > class_boxes_list_1(2);
        detect_ped_cyc_kernel<<<NUM_ANCHOR_Y_PED_CYC_INDS_, NUM_ANCHOR_X_PED_CYC_INDS_>>>
                (nn_out, GRID_X_, 
                NUM_ANCHOR_X_PED_CYC_INDS_, NUM_ANCHOR_C_PED_CYC_INDS_,
                sigmoid_score_ped_cyc_threshold_, MIN_Z_, MAX_Z_,
                ANCHOR_PED_W_, ANCHOR_PED_L_,
                ANCHOR_CYC_W_, ANCHOR_CYC_L_,
                dev_ped_cyc_filter_count_, dev_ped_cyc_filter_box_);
        int host_filter_ped_cyc_count[1];
        GPU_CHECK(cudaMemcpy(host_filter_ped_cyc_count, dev_ped_cyc_filter_count_,
                             sizeof(int), cudaMemcpyDeviceToHost));
        GPU_CHECK(cudaMemcpy(host_ped_cyc_bbox_, dev_ped_cyc_filter_box_, 
                host_filter_ped_cyc_count[0] * 10 * sizeof(float), cudaMemcpyDeviceToHost));
        bounding_box_ -> BBoxToList(2, host_ped_cyc_bbox_, 
                                    host_filter_ped_cyc_count[0], class_boxes_list_1);
        bounding_box_ -> NonMaxSuppression(2, class_boxes_list_1, result);
    }
}
