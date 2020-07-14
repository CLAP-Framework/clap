#ifndef POSTPROCESS_CUDA_H
#define POSTPROCESS_CUDA_H
#include <iostream>
#include <vector>
#include "common.h"
#include "postprocess.h"
#include <memory>

class PostprocessCuda {
private:
	const float MIN_Z_;
	const float MAX_Z_;
	const int GRID_X_;
	const int NUM_ANCHOR_X_CAR_INDS_;
	const int NUM_ANCHOR_Y_CAR_INDS_;
	const int NUM_ANCHOR_C_CAR_INDS_;
	const int NUM_ANCHOR_X_PED_CYC_INDS_;
	const int NUM_ANCHOR_Y_PED_CYC_INDS_;
	const int NUM_ANCHOR_C_PED_CYC_INDS_;
	const float ANCHOR_CAR_W_;
	const float ANCHOR_CAR_L_;
	const float ANCHOR_PED_W_;
	const float ANCHOR_PED_L_;
	const float ANCHOR_CYC_W_;
	const float ANCHOR_CYC_L_;
	const float score_car_threshold_;
	const float score_ped_cyc_threshold_;
	float sigmoid_score_car_threshold_;
	float sigmoid_score_ped_cyc_threshold_;
	const float nms_threshold_;
	const int NUM_OUTPUT_BOX_CAR_FEATURE_;
	const int NUM_OUTPUT_BOX_PED_CYC_FEATURE_;

	float *host_car_bbox_;
	float *host_ped_cyc_bbox_;
	int *dev_car_filter_count_ = nullptr;
    float *dev_car_filter_box_ = nullptr;
    int32_t nbytes_car_filter_count_;

    int *dev_ped_cyc_filter_count_ = nullptr;
    float *dev_ped_cyc_filter_box_ = nullptr;
    int32_t nbytes_ped_cyc_filter_count_;
    std::shared_ptr<PostProcess> bounding_box_;

public:
	/**
	 * @brief Constructor
	 * @param[in] FLOAT_MIN The lowest float value
	 * @param[in] FLOAT_MAX The maximum float value
	 * @param[in] NUM_ANCHOR_X_INDS Number of x-indexes for anchors
	 * @param[in] NUM_ANCHOR_Y_INDS Number of y-indexes for anchors
	 * @param[in] NUM_ANCHOR_R_INDS Number of rotation-indexes for anchors
	 * @param[in] score_threshold Score threshold for filtering output
	 * @param[in] NUM_THREADS Number of threads when launching cuda kernel
	 * @param[in] nms_overlap_threshold IOU threshold for NMS
	 * @param[in] NUM_BOX_CORNERS Number of box's corner
	 * @param[in] NUM_OUTPUT_BOX_FEATURE Number of output box's feature
	 * @details Captital variables never change after the compile, non-capital variables could be changed through rosparam
	 */
	PostprocessCuda(const float MIN_Z, const float MAX_Z,
		const int GRID_X, const int NUM_ANCHOR_X_CAR_INDS, 
		const int NUM_ANCHOR_Y_CAR_INDS, const int NUM_ANCHOR_C_CAR_INDS,
		const int NUM_ANCHOR_X_PED_CYC_INDS_, const int NUM_ANCHOR_Y_PED_CYC_INDS_,
		const int NUM_ANCHOR_C_PED_CYC_INDS_,
		const float ANCHOR_CAR_W, const float ANCHOR_CAR_L, 
		const float ANCHOR_PED_W_, const float ANCHOR_PED_L_, 
		const float ANCHOR_CYC_W_, const float ANCHOR_CYC_L_,
		const float score_car_threshold, const float score_ped_cyc_threshold,
		const float nms_threshold,
		const int NUM_OUTPUT_BOX_CAR_FEATURE,
		const int NUM_OUTPUT_BOX_PED_CYC_FEATURE);
  
	~PostprocessCuda();

	void doPostprocessCuda(const std::string &label,
						   const float *nn_car_out, 
						//    float *filtered_car_box,
						//    int *filter_car_count,
						   const int NUM_THREADS,
						   std::vector<BoundingBox> &result);
};

#endif