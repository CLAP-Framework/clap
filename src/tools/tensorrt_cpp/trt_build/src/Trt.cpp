#include "Trt.h"
#include "utils.h"
#include "spdlog/spdlog.h"

#include <string>
#include <vector>
#include <iostream>
#include <cassert>
#include <fstream>
#include <memory>
#include <chrono>

#include "NvInfer.h"
#include "NvCaffeParser.h"
#include "NvOnnxParser.h"
#include "NvUffParser.h"
#include "logger.h"
#include "config.h"

Trt::Trt() {
    // TrtPluginParams params;
    // mPluginFactory = new PluginFactory(params);
}

// Trt::Trt(TrtPluginParams params) {
//     mPluginFactory = new PluginFactory(params);
// }

Trt::~Trt() {
    // if(mPluginFactory != nullptr) {
    //     delete mPluginFactory;
    //     mPluginFactory = nullptr;
    // }
    if(mContext != nullptr) {
        mContext->destroy();
        mContext = nullptr;
    }
    if(mEngine !=nullptr) {
        mEngine->destroy();
        mEngine = nullptr;
    }
    for(size_t i=0;i<mBinding.size();i++) {
        safeCudaFree(mBinding[i]);
    }
}

void Trt::CreateEngine(
        const std::string& prototxt, 
        const std::string& caffeModel,
        const std::string& engineFile,
        const std::vector<std::string>& outputBlobName,
        int maxBatchSize,
        int mode) {
    mRunMode = mode;
    spdlog::info("prototxt: {}",prototxt);
    spdlog::info("caffeModel: {}",caffeModel);
    spdlog::info("engineFile: {}",engineFile);
    if(!BuildEngineWithCaffe(prototxt,caffeModel,engineFile,outputBlobName,maxBatchSize)) {
        spdlog::error("error: build engine failed");
        return;
    } else {
        spdlog::info("caffe model build engine succeed!");
    }
}

void Trt::CreateEngine(
        const std::string& onnxModel,
        const std::string& engineFile,
        const std::vector<std::string>& customOutput,
        int maxBatchSize,
        int mode) {
    mRunMode = mode;
    spdlog::info("onnxModel: {}", onnxModel);
    spdlog::info("engineFile: {}",engineFile);
    if(!BuildEngineWithOnnx(onnxModel,engineFile,customOutput,maxBatchSize)) {
        spdlog::error("error: could not deserialize or build engine");
        return;
    } else {
        spdlog::info("onnx model build engine succeed!");
    }
}

void Trt::CreateEngine(
        const std::string& uffModel,
        const std::string& engineFile,
        const std::vector<std::string>& inputTensorNames,
        const std::vector<std::vector<int>>& inputDims,
        const std::vector<std::string>& outputTensorNames,
        int maxBatchSize,
        int mode,
        const std::vector<std::vector<float>>& calibratorData) {
    if(!BuildEngineWithUff(uffModel,engineFile,inputTensorNames,inputDims, outputTensorNames,calibratorData, maxBatchSize)) {
        spdlog::error("error: could not deserialize or build engine");
        return;
    } else {
        spdlog::info("uff model build engine succeed!");
    }
}

void Trt::SaveEngine(const std::string& fileName) {
    if(fileName == "") {
        spdlog::warn("empty engine file name, skip save");
        return;
    }
    if(mEngine != nullptr) {
        spdlog::info("save engine to {}...",fileName);
        nvinfer1::IHostMemory* data = mEngine->serialize();
        std::ofstream file;
        file.open(fileName,std::ios::binary | std::ios::out);
        if(!file.is_open()) {
            spdlog::error("read create engine file {} failed",fileName);
            return;
        }
        file.write((const char*)data->data(), data->size());
        file.close();
        data->destroy();
    } else {
        spdlog::error("engine is empty, save engine failed");
    }
}

void Trt::BuildEngine(nvinfer1::IBuilder* builder,
                      nvinfer1::INetworkDefinition* network,
                      int maxBatchSize,
                      int mode) {
    #ifdef X86
    nvinfer1::IBuilderConfig* config = builder->createBuilderConfig();
    #endif

    ////INT8
    if (mode == 2) {
        spdlog::info("Set int8 inference mode");
    }
    ////FP16
    if (mode == 1) {
        spdlog::info("Set fp16 inference mode");
        if (!builder->platformHasFastFp16()) {
            spdlog::warn("The platform do not has fast for fp16");
        } else {
            #ifdef X86
            config->setFlag(nvinfer1::BuilderFlag::kFP16);
            #else
            // builder->setHalf2Mode(true);
            builder->setFp16Mode(true);
            #endif
        }
    }

    // Build the engine
    builder->setMaxBatchSize(maxBatchSize);
    #ifdef X86
    config->setMaxWorkspaceSize(1 << gMaxWorkspaceSize);
    #else
    builder->setMaxWorkspaceSize(1 << gMaxWorkspaceSize);
    #endif

    if (gCnnSeg) {
        /** add slice and sigmoid */
        //  binding bindIndex: 1, name: deconv0, size in byte: 19660800
        //  binding dims with 3 dimemsion  12 x 640 x 640 
        /** unmark output */
        auto deconv0_output = network->getOutput(0);
        nvinfer1::Dims deconv0_dims = deconv0_output->getDimensions();
        network->unmarkOutput(*deconv0_output);
        /** category */
        auto slice_category = network->addSlice(*deconv0_output,
                nvinfer1::DimsCHW{0, 0, 0}, 
                nvinfer1::DimsCHW{1, deconv0_dims.d[1], deconv0_dims.d[2]}, 
                nvinfer1::DimsCHW{1, 1, 1});
        slice_category->getOutput(0)->setName("category_pt");
        auto act_all_category_score = network->addActivation(
                *slice_category->getOutput(0), 
                nvinfer1::ActivationType::kSIGMOID); 
        act_all_category_score->getOutput(0)->setName("all_category_score");
        auto slice_mask = network->addSlice(*network->getInput(0),
                nvinfer1::DimsCHW{7, 0, 0}, 
                nvinfer1::DimsCHW{1, deconv0_dims.d[1], deconv0_dims.d[2]}, 
                nvinfer1::DimsCHW{1, 1, 1});
        slice_mask->getOutput(0)->setName("mask");
        auto eltwise_category_score = network->addElementWise(
                *slice_mask->getOutput(0), 
                *act_all_category_score->getOutput(0), 
                nvinfer1::ElementWiseOperation::kPROD);
        eltwise_category_score->getOutput(0)->setName("category_score");
        network->markOutput(*eltwise_category_score->getOutput(0));
        /** instance */
        auto slice_instance = network->addSlice(*deconv0_output,
            nvinfer1::DimsCHW{1, 0, 0}, 
            nvinfer1::DimsCHW{2, deconv0_dims.d[1], deconv0_dims.d[2]}, 
            nvinfer1::DimsCHW{1, 1, 1});
        slice_instance->getOutput(0)->setName("instance_pt");
        network->markOutput(*slice_instance->getOutput(0));
        /** confidence */
        auto slice_confidence = network->addSlice(*deconv0_output,
            nvinfer1::DimsCHW{3, 0, 0}, 
            nvinfer1::DimsCHW{1, deconv0_dims.d[1], deconv0_dims.d[2]}, 
            nvinfer1::DimsCHW{1, 1, 1});
        slice_confidence->getOutput(0)->setName("confidence_pt");
        auto act_confidence_score = network->addActivation(
                *slice_confidence->getOutput(0), 
                nvinfer1::ActivationType::kSIGMOID); 
        act_confidence_score->getOutput(0)->setName("confidence_score");
        network->markOutput(*act_confidence_score->getOutput(0));
        /** classify */
        auto slice_classify = network->addSlice(*deconv0_output,
            nvinfer1::DimsCHW{4, 0, 0}, 
            nvinfer1::DimsCHW{5, deconv0_dims.d[1], deconv0_dims.d[2]}, 
            nvinfer1::DimsCHW{1, 1, 1});
        slice_classify->getOutput(0)->setName("classify_pt");
        auto act_classify_score = network->addActivation(
                *slice_classify->getOutput(0), 
                nvinfer1::ActivationType::kSIGMOID); 
        act_classify_score->getOutput(0)->setName("classify_score");
        network->markOutput(*act_classify_score->getOutput(0));
        /** heading */
        auto slice_heading = network->addSlice(*deconv0_output,
            nvinfer1::DimsCHW{9, 0, 0}, 
            nvinfer1::DimsCHW{2, deconv0_dims.d[1], deconv0_dims.d[2]}, 
            nvinfer1::DimsCHW{1, 1, 1});
        slice_heading->getOutput(0)->setName("heading_pt");
        network->markOutput(*slice_heading->getOutput(0));
        /** height */
        auto slice_height = network->addSlice(*deconv0_output,
            nvinfer1::DimsCHW{11, 0, 0}, 
            nvinfer1::DimsCHW{1, deconv0_dims.d[1], deconv0_dims.d[2]}, 
            nvinfer1::DimsCHW{1, 1, 1});
        slice_height->getOutput(0)->setName("height_pt");
        network->markOutput(*slice_height->getOutput(0));
    }

    
    #ifdef X86
    mEngine = builder -> buildEngineWithConfig(*network, *config);
    assert(mEngine != nullptr);
    config->destroy();
    #else 
    mEngine = builder->buildCudaEngine(*network);
    assert(mEngine);
    #endif
}

bool Trt::BuildEngineWithCaffe(const std::string& prototxt, 
                        const std::string& caffeModel,
                        const std::string& engineFile,
                        const std::vector<std::string>& outputBlobName,
                        int maxBatchSize) {
    mBatchSize = maxBatchSize;
    spdlog::info("build caffe engine with {} and {}", prototxt, caffeModel);
    nvinfer1::IBuilder* builder = nvinfer1::createInferBuilder(mLogger);
    assert(builder != nullptr);
    // NetworkDefinitionCreationFlag::kEXPLICIT_BATCH 
    nvinfer1::INetworkDefinition* network = builder->createNetwork();
    assert(network != nullptr);
    nvcaffeparser1::ICaffeParser* parser = nvcaffeparser1::createCaffeParser();

    // Notice: change here to costom data type
    nvinfer1::DataType type = mRunMode==1 ? nvinfer1::DataType::kHALF : nvinfer1::DataType::kFLOAT;
    const nvcaffeparser1::IBlobNameToTensor* blobNameToTensor = parser->parse(prototxt.c_str(), caffeModel.c_str(),
                                                                              *network, type);
    for(auto& s : outputBlobName) {
        network->markOutput(*blobNameToTensor->find(s.c_str()));
    }
    spdlog::info("Number of network layers: {}",network->getNbLayers());
    spdlog::info("Number of input: ", network->getNbInputs());
    std::cout << "Input layer: " << std::endl;
    for(int i = 0; i < network->getNbInputs(); i++) {
        std::cout << network->getInput(i)->getName() << " : ";
        nvinfer1::Dims dims = network->getInput(i)->getDimensions();
        for(int j = 0; j < dims.nbDims; j++) {
            std::cout << dims.d[j] << "x"; 
        }
        std::cout << "\b "  << std::endl;
    }
    spdlog::info("Number of output: {}",network->getNbOutputs());
    std::cout << "Output layer: " << std::endl;
    for(int i = 0; i < network->getNbOutputs(); i++) {
        std::cout << network->getOutput(i)->getName() << " : ";
        nvinfer1::Dims dims = network->getOutput(i)->getDimensions();
        for(int j = 0; j < dims.nbDims; j++) {
            std::cout << dims.d[j] << "x"; 
        }
        std::cout << "\b " << std::endl;
    }
    spdlog::info("parse network done");

    BuildEngine(builder, network, maxBatchSize, mRunMode);

    spdlog::info("serialize engine to {}", engineFile);
    SaveEngine(engineFile);
    
    builder->destroy();
    network->destroy();
    parser->destroy();
    return true;
}

bool Trt::BuildEngineWithOnnx(const std::string& onnxModel,
                      const std::string& engineFile,
                      const std::vector<std::string>& customOutput,
                      int maxBatchSize) {
    mBatchSize = maxBatchSize;
    spdlog::info("build onnx engine from {}...",onnxModel);
    nvinfer1::IBuilder* builder = nvinfer1::createInferBuilder(mLogger);
    assert(builder != nullptr);
    // NetworkDefinitionCreationFlag::kEXPLICIT_BATCH 
    #ifdef X86
    const auto explicitBatch = 1U << static_cast<uint32_t>(nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
    nvinfer1::INetworkDefinition* network = builder->createNetworkV2(explicitBatch);
    #else
    nvinfer1::INetworkDefinition* network = builder->createNetwork();
    #endif
    assert(network != nullptr);
    nvonnxparser::IParser* parser = nvonnxparser::createParser(*network, mLogger);
    if(!parser->parseFromFile(onnxModel.c_str(), static_cast<int>(nvinfer1::ILogger::Severity::kWARNING))) {
        spdlog::error("error: could not parse onnx engine");
        return false;
    }

    for(int i=0;i<network->getNbLayers();i++) {
        nvinfer1::ILayer* custom_output = network->getLayer(i);
        for(int j=0;j<custom_output->getNbInputs();j++) {
            nvinfer1::ITensor* input_tensor = custom_output->getInput(j);
            std::cout << input_tensor->getName() << " ";
        }
        std::cout << " -------> ";
        for(int j=0;j<custom_output->getNbOutputs();j++) {
            nvinfer1::ITensor* output_tensor = custom_output->getOutput(j);
            std::cout << output_tensor->getName() << " ";
        }
        std::cout << std::endl;
    }  
    if(customOutput.size() > 0) {
        spdlog::info("unmark original output...");
        for(int i=0;i<network->getNbOutputs();i++) {
            nvinfer1::ITensor* origin_output = network->getOutput(i);
            network->unmarkOutput(*origin_output);
        }
        spdlog::info("mark custom output...");
        for(int i=0;i<network->getNbLayers();i++) {
            nvinfer1::ILayer* custom_output = network->getLayer(i);
            nvinfer1::ITensor* output_tensor = custom_output->getOutput(0);
            for(size_t j=0; j<customOutput.size();j++) {
                std::string layer_name(output_tensor->getName());
                if(layer_name == customOutput[j]) {
                    network->markOutput(*output_tensor);
                    break;
                }
            }
        }    
    }
    BuildEngine(builder, network, maxBatchSize, mRunMode);

    spdlog::info("serialize engine to {}", engineFile);
    SaveEngine(engineFile);

    builder->destroy();
    network->destroy();
    parser->destroy();
    return true;
}

bool Trt::BuildEngineWithUff(const std::string& uffModel,
                      const std::string& engineFile,
                      const std::vector<std::string>& inputTensorNames,
                      const std::vector<std::vector<int>>& inputDims,
                      const std::vector<std::string>& outputTensorNames,
                      const std::vector<std::vector<float>>& calibratorData,
                      int maxBatchSize) {
    // mBatchSize = maxBatchSize;
    // spdlog::info("build uff engine with {}...", uffModel);
    // nvinfer1::IBuilder* builder = nvinfer1::createInferBuilder(mLogger);
    // assert(builder != nullptr);
    // // NetworkDefinitionCreationFlag::kEXPLICIT_BATCH 
    // nvinfer1::INetworkDefinition* network = builder->createNetworkV2(0);
    // assert(network != nullptr);
    // nvuffparser::IUffParser* parser = nvuffparser::createUffParser();
    // assert(parser != nullptr);
    // assert(inputTensorNames.size() == inputDims.size());
    // //parse input
    // for(size_t i=0;i<inputTensorNames.size();i++) {
    //     nvinfer1::Dims dim;
    //     dim.nbDims = inputDims[i].size();
    //     for(int j=0;j<dim.nbDims;j++) {
    //         dim.d[j] = inputDims[i][j];
    //     }
    //     parser->registerInput(inputTensorNames[i].c_str(), dim, nvuffparser::UffInputOrder::kNCHW);
    // }
    // //parse output
    // for(size_t i=0;i<outputTensorNames.size();i++) {
    //     parser->registerOutput(outputTensorNames[i].c_str());
    // }
    // if(!parser->parse(uffModel.c_str(), *network, nvinfer1::DataType::kFLOAT)) {
    //     spdlog::error("error: parse model failed");
    // }
    // BuildEngine(builder, network, calibratorData, maxBatchSize, mRunMode);
    // spdlog::info("serialize engine to {}", engineFile);
    // SaveEngine(engineFile);
    
    // builder->destroy();
    // network->destroy();
    // parser->destroy();
    return true;
}