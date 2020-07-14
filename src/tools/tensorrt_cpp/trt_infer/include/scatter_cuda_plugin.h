#ifndef SCATTER_PLUGIN_H_
#define SCATTER_PLUGIN_H_

#include "NvInfer.h"
#include "NvInferPlugin.h"
#include "spdlog.h"
#ifdef X86
#include "NvInferRuntime.h"
#include "NvInferRuntimeCommon.h"
#endif

#ifdef X86  //TensorRT 7.0 version
class ScatterPlugin : public nvinfer1::IPluginV2DynamicExt {
public:
    ScatterPlugin();
    /* 这个就是从序列化数据里面恢复plugin的相关数据,另一个函数serialize将类的数据写
     * 入到序列化数据里面.在IPluginCreator::deserializePlugin里面会调用到这个函数,
     * 注意写的顺序跟读的顺序必须是一样的.
     */
    ScatterPlugin(const void* data, size_t length);
    ~ScatterPlugin() override = default;
    // 返回输出tensor的数量
    virtual int getNbOutputs() const override;
    // 返回输出tensor的维度
    virtual nvinfer1::DimsExprs getOutputDimensions(int outputIndex, 
                const nvinfer1::DimsExprs* inputs, int nbInputs, nvinfer1::IExprBuilder& exprBuilder) override;
    virtual nvinfer1::DataType getOutputDataType(int index, 
        const nvinfer1::DataType* inputTypes, int nbInputs) const override;
    
    /* 查询对应的datatype和format是否支持, 这个取决于你的自定义层实现是否支持
     */
    virtual bool supportsFormatCombination(int pos, const nvinfer1::PluginTensorDesc* inOut, 
                                           int nbInputs, int nbOutputs) override;
    virtual void configurePlugin(const nvinfer1::DynamicPluginTensorDesc* in, int nbInputs, 
                                 const nvinfer1::DynamicPluginTensorDesc* out, int nbOutputs) override;

    // 获得该层所需的临时显存大小
    virtual size_t getWorkspaceSize(const nvinfer1::PluginTensorDesc* inputs, int nbInputs, 
                                    const nvinfer1::PluginTensorDesc* outputs, int nbOutputs) const override;

    // 对该层进行初始化，在engine创建时被调用。
    virtual int initialize() override;
    // 获得该层进行serialization操作所需要的内存大小
    virtual size_t getSerializationSize() const override;
    /*序列化你的自定义插件到buffer,保证write的顺序和read的顺序是
     *       一样的,不然在反序列化的时候就会报错
     */
    virtual void serialize(void* buffer) const override;
    // 释放内存和显存
    virtual void terminate() override;
    // 执行该层
    virtual int enqueue(const nvinfer1::PluginTensorDesc* inputDesc, const nvinfer1::PluginTensorDesc* outputDesc, 
                        const void* const* inputs, void* const* outputs, void* workspace, 
                        cudaStream_t stream) override;

    virtual const char* getPluginType() const override;

    virtual const char* getPluginVersion() const override;

    virtual void destroy() override;

    nvinfer1::IPluginV2DynamicExt* clone() const override;

    virtual void setPluginNamespace(const char* pluginNamespace) override {}

    virtual const char* getPluginNamespace() const override;

private:
    int mNbBatchSize, mNbInputChannels, mNbInputHeight, mNbInputWidth;
    nvinfer1::DataType mDataType{nvinfer1::DataType::kFLOAT};
    int dev_voxel_num , dev_gMaxVoxels;
};
#else  //TensorRT 5.1 version
class ScatterPlugin : public nvinfer1::IPluginV2 {
public:
    ScatterPlugin();
    /* 这个就是从序列化数据里面恢复plugin的相关数据,另一个函数serialize将类的数据写
     * 入到序列化数据里面.在IPluginCreator::deserializePlugin里面会调用到这个函数,
     * 注意写的顺序跟读的顺序必须是一样的.
     */
    ScatterPlugin(const void* data, size_t length);
    ~ScatterPlugin() override = default;
    // 返回输出tensor的数量
    virtual int getNbOutputs() const override;
    // 返回输出tensor的维度
    virtual nvinfer1::Dims getOutputDimensions(int index, const nvinfer1::Dims* inputs, int nbInputDims) override;
    /* 查询对应的datatype和format是否支持, 这个取决于你的自定义层实现是否支持
     */
    virtual bool supportsFormat(nvinfer1::DataType type, nvinfer1::PluginFormat format) const override;
    virtual void configureWithFormat(const nvinfer1::Dims* inputDims, int nbInputs, 
                                      const nvinfer1::Dims* outputDims, int nbOutputs,
                                      nvinfer1::DataType type, nvinfer1::PluginFormat format, 
                                      int maxBatchSize) override;
    // 获得该层所需的临时显存大小
    virtual size_t getWorkspaceSize(int maxBatchSize) const override;
    // 对该层进行初始化，在engine创建时被调用。
    virtual int initialize() override;
    // 获得该层进行serialization操作所需要的内存大小
    virtual size_t getSerializationSize() const override;
    /*序列化你的自定义插件到buffer,保证write的顺序和read的顺序是
     *       一样的,不然在反序列化的时候就会报错
     */
    virtual void serialize(void* buffer) const override;
    // 释放内存和显存
    virtual void terminate() override;
    // 执行该层
    virtual int enqueue(int batchSize, const void*const * inputs, void** outputs,
                        void* workspace, cudaStream_t stream) override;

    virtual const char* getPluginType() const override;

    virtual const char* getPluginVersion() const override;

    virtual void destroy() override;

    virtual nvinfer1::IPluginV2* clone() const override;

    virtual void setPluginNamespace(const char* pluginNamespace) override {}

    virtual const char* getPluginNamespace() const override;

private:
    int mNbInputChannels, mNbInputHeight, mNbInputWidth;
    nvinfer1::DataType mDataType{nvinfer1::DataType::kFLOAT};
    int dev_voxel_num , dev_gMaxVoxels;
};
#endif

class ScatterPluginCreator : public nvinfer1::IPluginCreator {
public:

    virtual const char* getPluginName() const override;

    virtual const char* getPluginVersion() const override;

    virtual const nvinfer1::PluginFieldCollection* getFieldNames() override;

    virtual nvinfer1::IPluginV2* createPlugin(const char* name, const nvinfer1::PluginFieldCollection *fc) override;

    virtual nvinfer1::IPluginV2* deserializePlugin(const char* name, const void* serialData, size_t serialLenth) override;

    virtual void setPluginNamespace(const char* pluginNamespace) override {}

    virtual const char* getPluginNamespace() const override;

};

#endif