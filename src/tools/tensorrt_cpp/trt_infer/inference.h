#ifndef INFERENCE_H___
#define INFERENCE_H___

#include <iostream>
#include <vector>

namespace novauto {
namespace tensorrt {
namespace inference {

class Inference {
public:
	Inference() = default;
	virtual ~Inference() = default;

	/**
     * @description: set input data to the device memory from host memory.
	 * @bindIndex binding data index.
	 * @shapes the format is vector container(host).
     */
	virtual void SetInputTensor(const int &bindIndex, std::vector<float> &shapes) = 0;
     
     /**
     * @description: set input data to the device memory.
	 * @bindIndex binding data index.
	 * @shapes the format is float pointer(host).
      * @dataSize the counts of the input data.
      * @isHostToDevice
      *   True: copy data to device memory from host memory;
      *   False: the data in device memory, don't need copy;
     */
	virtual void SetInputTensor(const int &bindIndex, float *shapes, \
                                 int dataSize, bool isHostToDevice) = 0;

	/**
     * @description: do inference on engine context, make sure you already copy your data to device memory.
     */
	virtual void Infer() = 0;

	/**
     * @description: get output data to the host memory from device memory.
     */
	virtual void GetOutputTensor(const int &bindIndex, std::vector<float> &shapes) = 0;

     /**
     * @description: get output data to the host memory from device memory.
     *    You should alloc in the host memory by dataSize.
     *  @outputData the pointer to host memory. 
     *  @dataSize the counts of the output data.
     */
     virtual void GetOutputTensor(const int &bindIndex, float *outputData, int dataSize) = 0;
     
     /**
     * @description: get binding data pointer in device. for example if you want to do some post processing
     *               on inference output but want to process them in gpu directly for efficiency, you can
     *               use this function to avoid extra data io
     * @return: pointer point to device memory.
     */
    virtual void* GetBindingPtr(int bindIndex) const = 0;

	virtual int GetDevice() const = 0;
	/**
     * @description: get max batch size of build engine.
     * @return: max batch size of build engine.
     */
    virtual int GetMaxBatchSize() const = 0;

    /**
     * @description: get binding data size in byte, so maybe you need to divide it by sizeof(T) where T is data type
     *               like float.
     * @return: size in byte.
     */
    virtual size_t GetBindingSize(int bindIndex) const = 0;

    /**
     * @description: get binding dimemsions
     * @return: binding dimemsions, see https://docs.nvidia.com/deeplearning/sdk/tensorrt-api/c_api/classnvinfer1_1_1_dims.html
     */
    virtual std::vector<int> GetBindingDims(int bindIndex) const = 0;

    /**
     * @description: get binding data type
     * @return: binding data type, see https://docs.nvidia.com/deeplearning/sdk/tensorrt-api/c_api/namespacenvinfer1.html#afec8200293dc7ed40aca48a763592217
	 * kFLOAT : FP32 format, 
	 * kHALF : FP16 format, 
	 * kINT8 :	quantized INT8 format,
	 * kINT32 : INT32 format,
	 * kBOOL : BOOL format.
     */
    virtual int GetBindingDataType(int bindIndex) const = 0;

protected:
	
};

}  // namespace inference
}  // namespace tensorrt
}  // namespace novauto

#endif