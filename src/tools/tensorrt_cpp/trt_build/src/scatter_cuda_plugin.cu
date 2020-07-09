#include <cstring>
#include <vector>
#include "cuda_runtime.h"
#include "cuda_fp16.h"

#include "NvInfer.h"
#include "NvInferPlugin.h"

#include "scatter_cuda_plugin.h" 
#include "plugin_utils.h"

#include "spdlog/spdlog.h"
#include <cassert>

static const char* G_SCATTER_TYPE = "Scatter";
static const char* G_SCATTER_NAME = "Scatter_TRT"; //plugin_name = plugin_type + plugin_namespace

// CUDA: use 512 threads per block
static const int CUDA_NUM_THREADS = 512;

// CUDA: number of blocks for threads.
inline int CAFFE_GET_BLOCKS(const int N) {
	return (N + CUDA_NUM_THREADS - 1) / CUDA_NUM_THREADS;
}

// CUDA: grid stride looping
#define CUDA_KERNEL_LOOP(i, n) \
	for (int i = blockIdx.x * blockDim.x + threadIdx.x; \
       i < (n); \
       i += blockDim.x * gridDim.x)

///////////Scatter CUDA Function/////////////
///Cuda kernel forward
template <typename Dtype>
__global__ void cuda_scatter_plugin(
		const Dtype* p1_output, Dtype* p2_input, const Dtype* coors,
		const Dtype *voxel_num, int gMaxVoxels, 
		int gWidth, int gHeight, int gChannel) {
	CUDA_KERNEL_LOOP(i, (int)(*voxel_num)) {
		int row = (int)coors[2*i] ;
		int col = (int)coors[2*i+1] ;
		for (int j = 0; j < gChannel; j++) {
			Dtype feature = p1_output[i + j * gMaxVoxels];
			p2_input[j * gHeight * gWidth + row * gWidth + col] = feature;
		}
	}
}

template <typename Dtype>
cudaError_t do_scatter_plugin(const Dtype *voxel_num, int gMaxVoxels, int gChannel, 
                              int gHeight, int gWidth, const Dtype* coors,
                              const Dtype* p1_output, Dtype* p2_input) {
	cudaMemset(p2_input, 0, gChannel * gHeight * gWidth * sizeof(Dtype));
	cuda_scatter_plugin<<<64,128>>>(p1_output, p2_input, coors, \
				voxel_num, gMaxVoxels, gWidth, gHeight, gChannel);
	cudaError_t err = cudaGetLastError();
	return err;
}

//////////////////Plugin Class///////////////
ScatterPlugin::ScatterPlugin(){
    
}

ScatterPlugin::ScatterPlugin(const void* data, size_t length){
    const char *d = static_cast<const char *>(data), *a = d;
    read<int>(d, mNbInputChannels);
    read<int>(d, mNbInputHeight);
    read<int>(d, mNbInputWidth);
    read<nvinfer1::DataType>(d, mDataType);
    ASSERT(d == a + length);
}

int ScatterPlugin::getNbOutputs() const {
	return 1;
}
#ifdef X86
nvinfer1::DimsExprs ScatterPlugin::getOutputDimensions(
		int outputIndex, const nvinfer1::DimsExprs* inputs, 
		int nbInputs, nvinfer1::IExprBuilder& exprBuilder) {
	nvinfer1::DimsExprs output;
	output.nbDims = 4;
	output.d[0] = exprBuilder.constant(1);
	output.d[1] = exprBuilder.constant(32);
	output.d[2] = exprBuilder.constant(640);
	output.d[3] = exprBuilder.constant(320);
	return output;
}

nvinfer1::DataType ScatterPlugin::getOutputDataType(int index, 
		const nvinfer1::DataType* inputTypes, int nbInputs) const {
	return mDataType;
}

bool ScatterPlugin::supportsFormatCombination(int pos, 
		const nvinfer1::PluginTensorDesc* inOut, 
		int nbInputs, int nbOutputs) {
	return ((inOut[pos].type == nvinfer1::DataType::kFLOAT) || 
	        (inOut[pos].type == nvinfer1::DataType::kHALF)) && 
		   inOut[pos].format == nvinfer1::TensorFormat::kLINEAR;
}

void ScatterPlugin::configurePlugin(
		const nvinfer1::DynamicPluginTensorDesc* in, int nbInputs, 
		const nvinfer1::DynamicPluginTensorDesc* out, int nbOutputs) {
	mNbInputChannels = 32; 
	mNbInputHeight = 640;
	mNbInputWidth = 320;
}

size_t ScatterPlugin::getWorkspaceSize(
		const nvinfer1::PluginTensorDesc* inputs, int nbInputs, 
		const nvinfer1::PluginTensorDesc* outputs, int nbOutputs) const {
	return 0;
}

int ScatterPlugin::enqueue(
		const nvinfer1::PluginTensorDesc* inputDesc, 
		const nvinfer1::PluginTensorDesc* outputDesc, 
		const void* const* inputs, void* const* outputs, void* workspace, 
		cudaStream_t stream) {
	if (mDataType == nvinfer1::DataType::kFLOAT) {
		CUDA_CHECK(do_scatter_plugin<float>(
			reinterpret_cast<const float *>(inputs[2]), \
			20000, mNbInputChannels,					\
			mNbInputHeight, mNbInputWidth,				\
			reinterpret_cast<const float *>(inputs[1]), \
			reinterpret_cast<const float *>(inputs[0]), \
			reinterpret_cast<float *>(outputs[0])));
	} else if (mDataType == nvinfer1::DataType::kHALF) {
		CUDA_CHECK(do_scatter_plugin<__half>(
			reinterpret_cast<const __half *>(inputs[2]), \
			20000, mNbInputChannels,					 \
			mNbInputHeight, mNbInputWidth,				 \
			reinterpret_cast<const __half *>(inputs[1]), \
			reinterpret_cast<const __half *>(inputs[0]), \
			reinterpret_cast<__half *>(outputs[0])));
	}
	return 0;
}

nvinfer1::IPluginV2DynamicExt* ScatterPlugin::clone() const {
	return new ScatterPlugin();
}

#else
nvinfer1::Dims ScatterPlugin::getOutputDimensions(int index, 
			const nvinfer1::Dims* inputs, int nbInputDims) {
	return nvinfer1::Dims3(32,640,320); 
}

bool ScatterPlugin::supportsFormat(nvinfer1::DataType type, nvinfer1::PluginFormat format) const {
	return ((type == nvinfer1::DataType::kFLOAT) || 
	        (type == nvinfer1::DataType::kHALF)) && 
		   format == nvinfer1::PluginFormat::kNCHW;
}

void ScatterPlugin::configureWithFormat(const nvinfer1::Dims* inputDims, int nbInputs, 
                                        const nvinfer1::Dims* outputDims, int nbOutputs,
                                        nvinfer1::DataType type, nvinfer1::PluginFormat format, 
                                        int maxBatchSize) {
	ASSERT((type == nvinfer1::DataType::kFLOAT) || (type == nvinfer1::DataType::kHALF)
	&& format == nvinfer1::PluginFormat::kNCHW);
	mNbInputChannels = 32; //It is more proper to set name as output channels rather than input.
	mNbInputHeight = 640;
	mNbInputWidth = 320;
	mDataType = type;
}

int ScatterPlugin::enqueue(int batchSize, const void*const * inputs, 
                           void** outputs, void* workspace, cudaStream_t stream) {
	if (mDataType == nvinfer1::DataType::kFLOAT) {
		CUDA_CHECK(do_scatter_plugin<float>(             \
			reinterpret_cast<const float *>(inputs[2]),  \
			20000, mNbInputChannels,                     \
			mNbInputHeight, mNbInputWidth,               \
			reinterpret_cast<const float *>(inputs[1]),  \
			reinterpret_cast<const float *>(inputs[0]),  \
			reinterpret_cast<float *>(outputs[0])));
	} else if (mDataType == nvinfer1::DataType::kHALF) {
		CUDA_CHECK(do_scatter_plugin<__half>(            \
			reinterpret_cast<const __half *>(inputs[2]), \
			20000, mNbInputChannels,                     \
			mNbInputHeight, mNbInputWidth,               \
			reinterpret_cast<const __half *>(inputs[1]), \
			reinterpret_cast<const __half *>(inputs[0]), \
			reinterpret_cast<__half *>(outputs[0])));   
	}
	return 0;
}

nvinfer1::IPluginV2* ScatterPlugin::clone() const {
	return new ScatterPlugin();
}

size_t ScatterPlugin::getWorkspaceSize(int maxBatchSize) const {
    return 0;
}
#endif

int ScatterPlugin::initialize() {
    return 0;
}


size_t ScatterPlugin::getSerializationSize() const{
    return sizeof(mNbInputChannels) + sizeof(mNbInputWidth) + \
	       sizeof(mNbInputHeight) + sizeof(mDataType); 
}


void ScatterPlugin::serialize(void *buffer) const {
	char *d = static_cast<char *>(buffer), *a = d;
	write(d, mNbInputChannels);
	write(d, mNbInputHeight);
	write(d, mNbInputWidth);
	write(d, mDataType);
	ASSERT(d == a + getSerializationSize());
}

void ScatterPlugin::terminate() {

}

const char *ScatterPlugin::getPluginType() const {
	return G_SCATTER_TYPE;
}

const char *ScatterPlugin::getPluginVersion() const {
	return G_PLUGIN_VERSION;
}

void ScatterPlugin::destroy() {

}

const char* ScatterPlugin::getPluginNamespace() const {
	return G_PLUGIN_NAMESPACE;
}

const char* ScatterPluginCreator::getPluginName() const {
	return G_SCATTER_NAME;
}

const char* ScatterPluginCreator::getPluginVersion() const {
	return G_PLUGIN_VERSION;
}

const nvinfer1::PluginFieldCollection* ScatterPluginCreator::getFieldNames() {
	return nullptr;
}

nvinfer1::IPluginV2* ScatterPluginCreator::createPlugin(
		const char* name, const nvinfer1::PluginFieldCollection* fc) {
	return nullptr;
}

nvinfer1::IPluginV2* ScatterPluginCreator::deserializePlugin(
	const char *layerName, const void *serialData, size_t serialLength) {
	std::string strName{layerName};
	std::transform(strName.begin(), strName.end(), strName.begin(), ::tolower);
	if (strName.find("Scatter") != std::string::npos ||
	    strName.find("scatter") != std::string::npos) {
		return (nvinfer1::IPluginV2*)(new ScatterPlugin(serialData, serialLength));
	} else {
		std::cout << "warning : " << layerName << std::endl;
		assert(0);
		return nullptr;
	}
}

const char* ScatterPluginCreator::getPluginNamespace() const {
	return G_PLUGIN_NAMESPACE;
}

REGISTER_TENSORRT_PLUGIN(ScatterPluginCreator);