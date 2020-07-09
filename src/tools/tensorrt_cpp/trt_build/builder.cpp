#include "Trt.h"
#include "logger.h"
#include "config.h"
#include <fstream>

int main(int argc, char *argv[]) {
    if (argc < 2) {
        spdlog::error("Please input the configuration file path.");
        return 0;
    }
    loadYamlFile(argv[1]);
    if (gModelType == "caffe") {
        Trt* caffe_net = new Trt();
        caffe_net->CreateEngine(gCaffePrototxt, gCaffeModel, \
                                gEngineName, gOutputNode, \
                                gMaxBatchSize, gDataType);
    } else if (gModelType == "onnx") {
        Trt* onnx_net = new Trt();
        onnx_net->CreateEngine(gOnnxModel, gEngineName, \
                               gOutputNode, gMaxBatchSize, gDataType);

    } else if (gModelType == "uff") {

    } else {

    }
    return 0;
}