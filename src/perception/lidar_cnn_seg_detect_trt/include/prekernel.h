#ifndef PREKERNEL_H
#define PREKERNEL_H

#include <cuda_runtime.h>
#include <device_launch_parameters.h>
// #include "struct.h"

struct PointXYZI {
	float x;
	float y;
	float z;
	float res0;
	float i;
	float res1;
	float res2;
	float res3;
	PointXYZI(float xx, float yy, float zz, float rr)
			: x(xx), y(yy), z(zz), i(rr){}
};
// __global__ void cuda_test(unsigned int *test_data, float* dev_logtable);
__global__ void cloud_distribution(const PointXYZI* dev_pc, int* dev_map_idx_, int dev_max_h,
                                   int dev_min_h, float dev_inv_res_x, float dev_inv_res_y,
                                   int dev_range, int dev_width, int dev_height,
                                   float* dev_max_height_data_, float* dev_top_intensity_data_,
                                   float* dev_mean_height_data_, float* dev_mean_intensity_data_, 
                                   float* dev_count_data_, unsigned int num, int* dev_mutex);
__global__ void cloud_average(float* dev_max_height_data_, float* dev_mean_height_data_,
		float* dev_count_data_, float* dev_mean_intensity_data_, 
		float* dev_nonempty_data_, unsigned int ch_size, float* dev_log_table_);
// __global__ void cloud_max_set(float* dev_count_data_,float* dev_max_height_data_);
// __global__ void cloud_mean_calculation(float* dev_count_data_, float* dev_mean_height_data_, float* dev_mean_intensity_data_,
//                                        float* dev_nonempty_data_);
// __global__ void cloud_count_update(float* dev_count_data_, unsigned int ch_size, float* dev_count_data_2);


__global__ void cloud_position_cal_dis(const PointXYZI* dev_pc, unsigned int pc_size, float dev_inv_res_x, 
	float dev_inv_res_y, float dev_range, int* dev_map_idx_,
    int dev_min_h, int dev_max_h, int dev_width, int dev_height);
__global__ void cloud_top_max_update(const PointXYZI* dev_pc, unsigned int pc_size, int* dev_map_idx_,
		float* dev_max_height_data_, float* dev_top_intensity_data_);
__global__ void cloud_other_cal(const PointXYZI* dev_pc, unsigned int pc_size, float* dev_mean_height_data_, 
								float* dev_mean_intensity_data_, float* dev_count_data_, int dev_range, int dev_width,
								int dev_height, int dev_max_h, int dev_min_h, float dev_inv_res_x,
								float dev_inv_res_y);
__global__ void cuda_mem_set(float* dev_men_, int mem_size, float val);
__device__ int dev_F2I(float val, float ori, float scale);
//__device__ int Pc2Pixel(float in_pc, float in_range, float out_size);
//__device__ int Pixel2Pc(int in_pixel, float in_size, float out_range);



#endif