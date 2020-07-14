#include "config.h"

std::string gModelType;
int gDataType;
int gMaxBatchSize;
int gMaxWorkspaceSize;
std::string gEngineName;
std::vector<std::string> gInputNode;
std::vector<std::string> gOutputNode;

//// caffe model
std::string gCaffePrototxt;
std::string gCaffeModel;
bool gCnnSeg;
//// onnx model
std::string gOnnxModel;
//// uff model
std::string gUffModel;

void loadYamlFile(const std::string &name) {
    YAML::Node node;
    try {
        node = YAML::LoadFile(name);
    } catch (std::exception e) {
        spdlog::error("Can't find configure file {} .", name);
    }
    // load parameter gDisplayIp
    GetParameter(node, "gModelType", gModelType);
    GetParameter(node, "gDataType", gDataType);
    GetParameter(node, "gMaxBatchSize", gMaxBatchSize);
    GetParameter(node, "gMaxWorkspaceSize", gMaxWorkspaceSize);
    // GetParameter(node, "gEngineName", gEngineName);
    GetParameter(node, "gInputNode", gInputNode);
    GetParameter(node, "gOutputNode", gOutputNode);
    if (gModelType == "caffe") {
        GetParameter(node, "gCaffePrototxt", gCaffePrototxt);
        GetParameter(node, "gCaffeModel", gCaffeModel);
        GetParameter(node, "gCnnSeg", gCnnSeg);
        if (gDataType == 0) {        //FP32
            gEngineName = gCaffeModel + "_FP32.trt";
        } else if (gDataType == 1) { //FP16
            gEngineName = gCaffeModel + "_FP16.trt";
        } else if (gDataType == 2) { //INT8
            gEngineName = gCaffeModel + "_INT8.trt";
        }
    } else if (gModelType == "onnx") {
        GetParameter(node, "gOnnxModel", gOnnxModel);
        if (gDataType == 0) {        //FP32
            gEngineName = gOnnxModel + "_FP32.trt";
        } else if (gDataType == 1) { //FP16
            gEngineName = gOnnxModel + "_FP16.trt";
        } else if (gDataType == 2) { //INT8
            gEngineName = gOnnxModel + "_INT8.trt";
        }
    } else if (gModelType == "uff") {
        GetParameter(node, "gUffModel", gUffModel);
        if (gDataType == 0) {        //FP32
            gEngineName = gUffModel + "_FP32.trt";
        } else if (gDataType == 1) { //FP16
            gEngineName = gUffModel + "_FP16.trt";
        } else if (gDataType == 2) { //INT8
            gEngineName = gUffModel + "_INT8.trt";
        }
    }
    spdlog::info("The {} model will be compiled to TRT model...", gModelType);
}




