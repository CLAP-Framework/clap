#include <cuda_runtime.h>
#include <device_launch_parameters.h>

#include "prekernel.h"

__device__ int dev_F2I(float val, float ori, float scale){
  return __float2int_rd((ori - val) * scale);
}

__global__ void cloud_distribution(const PointXYZI* dev_pc, int* dev_map_idx_, int dev_max_h,
		int dev_min_h, float dev_inv_res_x, float dev_inv_res_y,
		int dev_range, int dev_width, int dev_height,
		float* dev_max_height_data_, float* dev_top_intensity_data_,
		float* dev_mean_height_data_, float* dev_mean_intensity_data_, 
		float* dev_count_data_, unsigned int num, int* dev_mutex) {
     
	for(unsigned int i = blockIdx.x * blockDim.x + threadIdx.x; 
			i < (num);	i+= blockDim.x * gridDim.x) {
		PointXYZI point = dev_pc[i];

		if (point.z <= dev_min_h || point.z >= dev_max_h) {
			continue;
		}
		
		int pos_x = dev_F2I(point.y, dev_range, dev_inv_res_x);  // col
		int pos_y = dev_F2I(point.x, dev_range, dev_inv_res_y);  // row
		if (pos_x >= dev_width || pos_x < 0 || pos_y >= dev_height || pos_y < 0) {
			continue;
		}

		int idx = pos_y * dev_width + pos_x;
		float pz = point.z;
		float pi = point.i/255.0f;
		
		float temp = dev_max_height_data_[idx];
		dev_max_height_data_[idx] = fmaxf(temp,pz);
		__threadfence();
		if (dev_max_height_data_[idx] < pz) {
			int t_pi = __float_as_int(pi);
			atomicExch((int*)&dev_top_intensity_data_[idx], t_pi);
		}

		atomicAdd(&dev_mean_height_data_[idx], pz);
		atomicAdd(&dev_mean_intensity_data_[idx], pi);
		atomicAdd(&dev_count_data_[idx], 1.0f);
	} 
}

__global__ void cloud_average(float* dev_max_height_data_, float* dev_mean_height_data_,
		float* dev_count_data_, float* dev_mean_intensity_data_,
		float* dev_nonempty_data_, unsigned int ch_size, float* dev_log_table_ ) {       
	for(unsigned int index = blockIdx.x * blockDim.x + threadIdx.x; 
			index < (ch_size); index += blockDim.x * gridDim.x) {
		float count_data_ = dev_count_data_[index];
		float mean_height_data_ = dev_mean_height_data_[index];
		float mean_intensity_data_ = dev_mean_intensity_data_[index];
		///> The ESP value can be changed 
		if (count_data_ < 1e-6) {  
			dev_max_height_data_[index] = 0.0f;
		} else {
			mean_height_data_ /= count_data_;
			mean_intensity_data_ /= count_data_;
			dev_nonempty_data_[index] = 1.0f;
		}
		int count = (int)(count_data_);
		count_data_ = dev_log_table_[count];
		dev_count_data_[index] = count_data_;
		dev_mean_intensity_data_[index] = mean_intensity_data_;
		dev_mean_height_data_[index] = mean_height_data_;
	}
}

__global__ void cloud_top_max_update(const PointXYZI* dev_pc, unsigned int pc_size, 
		int* dev_map_idx_,
		float* dev_max_height_data_, float* dev_top_intensity_data_){
	for(unsigned int i = blockIdx.x * blockDim.x + threadIdx.x; 
			i < (pc_size); i+= blockDim.x * gridDim.x){
		int idx = dev_map_idx_[i];
		float pz = dev_pc[i].z;
		float pi = dev_pc[i].i;
		//Currently do normalization
		
		if (idx>=0) {
			if (dev_max_height_data_[idx] < pz) {
					dev_max_height_data_[idx] = pz;
					dev_top_intensity_data_[idx] = pi ;
			}			
		}

  }
}

__global__ void cloud_other_cal(const PointXYZI* dev_pc, unsigned int pc_size, float* dev_mean_height_data_, 
								float* dev_mean_intensity_data_, float* dev_count_data_, int dev_range, int dev_width,
								int dev_height, int dev_max_h, int dev_min_h, float dev_inv_res_x,
								float dev_inv_res_y){
	for(unsigned int i = blockIdx.x * blockDim.x + threadIdx.x; 
			i < (pc_size); i+= blockDim.x * gridDim.x){
		PointXYZI ptc = dev_pc[i];
		int dev_px = dev_F2I(ptc.y, dev_range, dev_inv_res_x);  // col
		int dev_py = dev_F2I(ptc.x, dev_range, dev_inv_res_y);  // row
		if (ptc.z <= dev_min_h || ptc.z >= dev_max_h || dev_px >= dev_width 
				|| dev_px < 0 || dev_py >= dev_height || dev_py < 0) {
			continue;
		}
		int idx = dev_py * dev_width + dev_px;
		float pz = ptc.z;
		float pi = ptc.i/255.0f;
		
		atomicAdd(&dev_mean_height_data_[idx], pz);
		atomicAdd(&dev_mean_intensity_data_[idx], pi);
		atomicAdd(&dev_count_data_[idx], 1.0f);
	}
}

__global__ void cuda_mem_set(float* dev_men_, int mem_size, float val) {
  #pragma unroll
	for(int index = blockIdx.x * blockDim.x + threadIdx.x; 
			index < (mem_size); index += blockDim.x * gridDim.x){
		dev_men_[index] = val;
	}
}