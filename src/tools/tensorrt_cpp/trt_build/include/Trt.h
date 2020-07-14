#ifndef TRT_HPP
#define TRT_HPP

#include <string>
#include <vector>
#include <iostream>
#include <numeric>
#include <algorithm>

#include "NvInfer.h"



class TrtLogger : public nvinfer1::ILogger {
    void log(Severity severity, const char* msg) override
    {
        // suppress info-level messages
        if (severity != Severity::kVERBOSE)
            std::cout << msg << std::endl;
    }
};

// struct TrtPluginParams {
//     // yolo-det layer
//     int yoloClassNum = 1; 
//     int yolo3NetSize = 416; // 416 or 608

//     // upsample layer
//     float upsampleScale = 2;
// };

// class PluginFactory;

class Trt {
public:
    /**
     * @description: default constructor, will initialize plugin factory with default parameters.
     */
    Trt();

    /**
     * @description: if you costomize some parameters, use this.
     */
    // Trt(TrtPluginParams params);

    ~Trt();

    /**
     * description: create engine from caffe prototxt and caffe model
     * @prototxt: caffe prototxt
     * @caffemodel: caffe model contain network parameters
     * @engineFile: serialzed engine file, if it does not exit, will build engine from
     *             prototxt and caffe model, which take about 1 minites, otherwise will
     *             deserialize enfine from engine file, which is very fast.
     * @outputBlobName: specify which layer is network output, find it in caffe prototxt
     * @calibratorData: use for int8 mode, calabrator data is a batch of sample input, 
     *                  for classification task you need around 500 sample input. and this
     *                  is for int8 mode
     * @maxBatchSize: max batch size while inference, make sure it do not exceed max batch
     *                size in your model
     * @mode: engine run mode, 0 for float32, 1 for float16, 2 for int8
     */
    void CreateEngine(
        const std::string& prototxt, 
        const std::string& caffeModel,
        const std::string& engineFile,
        const std::vector<std::string>& outputBlobName,
        int maxBatchSize,
        int mode);
    
    /**
     * @description: create engine from onnx model
     * @onnxModel: path to onnx model
     * @engineFile: path to saved engien file will be load or save, if it's empty them will not
     *              save engine file
     * @maxBatchSize: max batch size for inference.
     * @return: 
     */
    void CreateEngine(
        const std::string& onnxModel,
        const std::string& engineFile,
        const std::vector<std::string>& customOutput,
        int maxBatchSize,
        int mode);

    /**
     * @description: create engine from uff model
     * @uffModel: path to uff model
     * @engineFile: path to saved engien file will be load or save, if it's empty them will not
     *              save engine file
     * @inputTensorName: input tensor
     * @outputTensorName: output tensor
     * @maxBatchSize: max batch size for inference.
     * @return: 
     */
    void CreateEngine(
        const std::string& uffModel,
        const std::string& engineFile,
        const std::vector<std::string>& inputTensorName,
        const std::vector<std::vector<int>>& inputDims,
        const std::vector<std::string>& outputTensorName,
        int maxBatchSize,
        int mode,
        const std::vector<std::vector<float>>& calibratorData);

    std::vector<std::string> mBindingName;

protected:

    void BuildEngine(nvinfer1::IBuilder* builder,
                      nvinfer1::INetworkDefinition* network,
                      int maxBatchSize,
                      int mode);

    bool BuildEngineWithCaffe(const std::string& prototxt, 
                    const std::string& caffeModel,
                    const std::string& engineFile,
                    const std::vector<std::string>& outputBlobName,
                    int maxBatchSize);

    bool BuildEngineWithOnnx(const std::string& onnxModel,
                     const std::string& engineFile,
                     const std::vector<std::string>& customOutput,
                     int maxBatchSize);

    bool BuildEngineWithUff(const std::string& uffModel,
                      const std::string& engineFile,
                      const std::vector<std::string>& inputTensorName,
                      const std::vector<std::vector<int>>& inputDims,
                      const std::vector<std::string>& outputTensorName,
                      const std::vector<std::vector<float>>& calibratorData,
                      int maxBatchSize);

    /**
     * description: save engine to engine file
     */
    void SaveEngine(const std::string& fileName);

protected:
    TrtLogger mLogger;

    // tensorrt run mode 0:fp32 1:fp16 2:int8
    int mRunMode;

    nvinfer1::ICudaEngine* mEngine = nullptr;

    nvinfer1::IExecutionContext* mContext = nullptr;

    // PluginFactory* mPluginFactory;

    nvinfer1::IRuntime* mRuntime = nullptr;

    std::vector<void*> mBinding;

    std::vector<size_t> mBindingSize;

    std::vector<nvinfer1::Dims> mBindingDims;

    std::vector<nvinfer1::DataType> mBindingDataType;

    int mInputSize = 0;

    // batch size
    int mBatchSize; 
};

#endif