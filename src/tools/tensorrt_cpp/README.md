## TensorRT C++ 工具使用说明

**该工具包含两部分**：TRT模型的构建、TRT模型的调用API；

**已测试平台包括**：工控机（TensorRT 7.0.0.11）、Xavier（TensorRT 5.1.6.1）；

### 1、TRT模型的构建

针对TensorRT 7.0.0.11和5.1.6.1版本，将caffe、onnx、uff(暂未支持)模型构建为TRT模型。

**工程编译流程：**

```
cd trt_build
mkdir build
cd build
cmake ..
make
```

**工具使用：**

修改配置文件config.yaml

```
###### option:caffe, onnx, uff ######
gModelType: caffe
###### option:0--FP32, 1--FP16, 2--INT8 ######
gDataType: 1
gMaxBatchSize: 1
gMaxWorkspaceSize: 30  #2^30

gInputNode: [input]
###### caffe model ######
gCaffePrototxt: ../models/deploy.prototxt
gCaffeModel: ../models/deploy.caffemodel
gOutputNode: [deconv0]
gCnnSeg: True

###### onnx model ######
# gOnnxModel: ../models/part1.onnx
# gOutputNode: [pointpillars_part1/features]
# gOnnxModel: ../models/part2.onnx
# gOutputNode: [pointpillars_part2/features]

###### uff model ######
gUffModel: ../models/
```

模型构建

```
./trt_builder
```



### 2、TRT模型的调用API

统一接口，通过C++ API对TRT模型的调用，完成模型推理过程。

**API 说明**
```
/**@description: set input data to the device memory.
 * @bindIndex binding data index.
 * @shapes the host input data.
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
```

**工程编译流程：**

```
cd trt_infer
mkdir build
cd build
cmake ..  /  cmake -DBUILD_TEST=OFF ..(若无需编译测试模块)
make
```

**API调用：**

动态链接库：libtrt.so

头文件：inference.h和inference_factory.h

使用demo：

```
std::shared_ptr<Inference> inference;
inference.reset(CreateInferenceByName(
			    "TRTNet",\
			    gEngineName,\               //TRT模型路径
				gInputNode, gOutputNode));  //输入节点名称
inference -> SetInputTensor(j, input_data); //输出节点名称
inference -> Infer();
inference -> GetOutputTensor(j + gInputNode.size(), output_data[j]);
```