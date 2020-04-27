#include "trt_net.h"
#include "spdlog/spdlog.h"
#include <cassert>
#include <fstream>
#include <numeric>

namespace novauto {
namespace tensorrt {
namespace inference {

TRTNet::TRTNet(const std::string &net_file) {
    if(!DeserializeEngine(net_file)) {
        spdlog::error("error: could not deserialize or build engine");
    } else {
        spdlog::info("create execute context and malloc device memory...");
        InitEngine();
    }
}

TRTNet::~TRTNet() {
    if(mContext_ != nullptr) {
        mContext_->destroy();
        mContext_ = nullptr;
    }
    if(mEngine_ !=nullptr) {
        mEngine_->destroy();
        mEngine_ = nullptr;
    }
    for(size_t i = 0; i < mBinding_.size(); i++) {
        safeCudaFree(mBinding_[i]);
    }
}

void TRTNet::SetInputTensor(const int &bindIndex, std::vector<float> &shapes) {
    DataTransfer(shapes, bindIndex, true);
}

void TRTNet::Infer() {
    cudaEvent_t start,stop;
    float elapsedTime;
    //使用event计算时间
    cudaEventCreate(&start);    //创建event
    cudaEventCreate(&stop);  
    cudaEventRecord(start, 0);  //记录当前时间
    mContext_->execute(mBatchSize_, &mBinding_[0]);
    cudaEventRecord(stop, 0);
	cudaEventSynchronize(stop);
	cudaEventElapsedTime(&elapsedTime, start, stop); //计算时间差
    spdlog::info("net forward takes {} ms", elapsedTime);
}

void TRTNet::GetOutputTensor(const int &bindIndex, std::vector<float> &shapes) {
    DataTransfer(shapes, bindIndex, false);
}

void TRTNet::DataTransfer(std::vector<float>& data, int bindIndex, bool isHostToDevice) {
    if(isHostToDevice) {
        assert(data.size() * sizeof(float) == mBindingSize_[bindIndex]);
        CUDA_CHECK(cudaMemcpy(mBinding_[bindIndex], data.data(), mBindingSize_[bindIndex], cudaMemcpyHostToDevice));
    } else {
        data.resize(mBindingSize_[bindIndex]/sizeof(float));
        CUDA_CHECK(cudaMemcpy(data.data(), mBinding_[bindIndex], mBindingSize_[bindIndex], cudaMemcpyDeviceToHost));
    }
}

void TRTNet::InitEngine() {
    spdlog::info("init engine...");
    mContext_ = mEngine_->createExecutionContext();
    assert(mContext_ != nullptr);

    spdlog::info("malloc device memory");
    int nbBindings = mEngine_->getNbBindings();
    std::cout << "nbBingdings: " << nbBindings << std::endl;
    mBinding_.resize(nbBindings);
    mBindingSize_.resize(nbBindings);
    mBindingName_.resize(nbBindings);
    mBindingDims_.resize(nbBindings);
    mBindingDataType_.resize(nbBindings);
    for(int i=0; i< nbBindings; i++) {
        nvinfer1::Dims dims = mEngine_->getBindingDimensions(i);
        nvinfer1::DataType dtype = mEngine_->getBindingDataType(i);
        const char* name = mEngine_->getBindingName(i);
        int64_t totalSize = volume(dims) * mBatchSize_ * getElementSize(dtype);
        mBindingSize_[i] = totalSize;
        mBindingName_[i] = name;
        mBindingDims_[i] = dims;
        mBindingDataType_[i] = dtype;
        if(mEngine_->bindingIsInput(i)) {
            spdlog::info("input: ");
        } else {
            spdlog::info("output: ");
        }
        spdlog::info("binding bindIndex: {}, name: {}, size in byte: {}",i,name,totalSize);
        spdlog::info("binding dims with {} dimemsion",dims.nbDims);
        for(int j=0;j<dims.nbDims;j++) {
            std::cout << dims.d[j] << " x ";
        }
        std::cout << "\b\b  "<< std::endl;
        mBinding_[i] = safeCudaMalloc(totalSize);
        if(mEngine_->bindingIsInput(i)) {
            mInputNodeSize_++;
        } else {
            mOutputNodeSize_++;
        }
    }
    mParser_ -> destroy();
    mNetwork_ -> destroy();
    mBuilder_ -> destroy();
}

bool TRTNet::DeserializeEngine(const std::string &net_file) {
    mBuilder_ = nvinfer1::createInferBuilder(mLogger_);
    mNetwork_ = mBuilder_ -> createNetwork();
    mParser_ = nvonnxparser::createParser(*mNetwork_, mLogger_);
    std::ifstream in(net_file.c_str(), std::ifstream::binary);
    if(in.is_open()) {
        spdlog::info("deserialize engine from {}", net_file);
        auto const start_pos = in.tellg();
        in.ignore(std::numeric_limits<std::streamsize>::max());
        size_t bufCount = in.gcount();
        in.seekg(start_pos);
        std::unique_ptr<char[]> engineBuf(new char[bufCount]);
        in.read(engineBuf.get(), bufCount);
        // initLibNvInferPlugins(&mLogger, "");
        mRuntime_ = nvinfer1::createInferRuntime(mLogger_);
        mEngine_ = mRuntime_->deserializeCudaEngine((void*)engineBuf.get(), bufCount, nullptr);
        assert(mEngine_ != nullptr);
        mBatchSize_ = mEngine_->getMaxBatchSize();
        spdlog::info("max batch size of deserialized engine: {}", mEngine_->getMaxBatchSize());
        mRuntime_->destroy();
        return true;
    }
    return false;
}

void* TRTNet::safeCudaMalloc(size_t memSize) {
    void* deviceMem;
    CUDA_CHECK(cudaMalloc(&deviceMem, memSize));
    if (deviceMem == nullptr) {
        std::cerr << "Out of memory" << std::endl;
        exit(1);
    }
    return deviceMem;
}

void TRTNet::safeCudaFree(void* deviceMem) {
    CUDA_CHECK(cudaFree(deviceMem));
}

int64_t TRTNet::volume(const nvinfer1::Dims& d) {
    return std::accumulate(d.d, d.d + d.nbDims, 1, std::multiplies<int64_t>());
}

unsigned int TRTNet::getElementSize(nvinfer1::DataType t) {
    switch (t)
    {
        case nvinfer1::DataType::kINT32: return 4;
        case nvinfer1::DataType::kFLOAT: return 4;
        case nvinfer1::DataType::kHALF: return 2;
        case nvinfer1::DataType::kINT8: return 1;
        default: throw std::runtime_error("Invalid DataType.");
    }
}

int TRTNet::GetDevice() const { 
    int* device = nullptr; //NOTE: memory leaks here
    CUDA_CHECK(cudaGetDevice(device));
    if(device != nullptr) {
        return device[0];
    } else {
        spdlog::error("Get Device Error");
        return -1;
    }
}

int TRTNet::GetMaxBatchSize() const{
    return mBatchSize_;
}

void* TRTNet::GetBindingPtr(int bindIndex) const {
    return mBinding_[bindIndex];
}

size_t TRTNet::GetBindingSize(int bindIndex) const {
    return mBindingSize_[bindIndex];
}

std::vector<int> TRTNet::GetBindingDims(int bindIndex) const {
    std::vector<int> shape;
    for(int j=0; j < mBindingDims_[bindIndex].nbDims; j++) {
        shape.push_back(mBindingDims_[bindIndex].d[j]);
    }
    return shape;
}

int TRTNet::GetBindingDataType(int bindIndex) const {
    return static_cast<int>(mBindingDataType_[bindIndex]);
}

} // namespace inference
} // namespace tensorrt
} // namespace novauto