#ifndef TRT_NET_H___
#define TRT_NET_H___

#include <iostream>

#include "inference.h"
#include "cuda_runtime.h"
#include "NvInfer.h"
#include "logger.h"
#include "NvOnnxParser.h"

namespace novauto {
namespace tensorrt {
namespace inference {

class TrtLogger : public nvinfer1::ILogger {
    void log(Severity severity, const char* msg) override
    {
        // suppress info-level messages
        if (severity != Severity::kVERBOSE)
            std::cout << msg << std::endl;
    }
};

class TRTNet : public Inference {
public:
    TRTNet(const std::string &net_file);
    virtual ~TRTNet();

    void SetInputTensor(const int &bindIndex, std::vector<float> &shapes) override;
    void Infer() override;
    void GetOutputTensor(const int &bindIndex, std::vector<float> &shapes) override;

    int GetDevice() const override;
    int GetMaxBatchSize() const override;
    void* GetBindingPtr(int bindIndex) const override;
    size_t GetBindingSize(int bindIndex) const override;
    std::vector<int> GetBindingDims(int bindIndex) const override;
    int GetBindingDataType(int bindIndex) const override;

    std::vector<std::string> mBindingName_;

protected:
    bool DeserializeEngine(const std::string &net_file);
    void InitEngine();
    void DataTransfer(std::vector<float>& data, int bindIndex, bool isHostToDevice);
    void* safeCudaMalloc(size_t memSize);
    void safeCudaFree(void* deviceMem);
    int64_t volume(const nvinfer1::Dims& d);
    unsigned int getElementSize(nvinfer1::DataType t);

protected:
    TrtLogger mLogger_;
    nvinfer1::IExecutionContext *mContext_ = nullptr;
    nvinfer1::ICudaEngine* mEngine_ = nullptr;
    nvinfer1::IRuntime* mRuntime_ = nullptr;
    int mBatchSize_; 
    std::vector<void*> mBinding_;
    std::vector<size_t> mBindingSize_;
    std::vector<nvinfer1::Dims> mBindingDims_;
    std::vector<nvinfer1::DataType> mBindingDataType_;
    int mInputNodeSize_ = 0;
    int mOutputNodeSize_ = 0;

    nvinfer1::IBuilder* mBuilder_;
    nvinfer1::INetworkDefinition* mNetwork_;
    nvonnxparser::IParser* mParser_;
};

#ifndef CUDA_CHECK
#define CUDA_CHECK(callstr)                                                                    \
    {                                                                                          \
        cudaError_t error_code = callstr;                                                      \
        if (error_code != cudaSuccess) {                                                       \
            std::cerr << "CUDA error " << error_code << " at " << __FILE__ << ":" << __LINE__ << std::endl; \
            exit(0);                                                                         \
        }                                                                                      \
    }
#endif

} // namespace inference
} // namespace tensorrt
} // namespace novauto

#endif