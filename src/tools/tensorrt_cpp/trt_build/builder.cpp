#include "Trt.h"
#include "logger.h"
#include "config.h"
#include <fstream>

int main(int argc, char *argv[]) {
    loadYamlFile("../config.yaml");
    /// caffe demo
    for(auto &s:gInputNode)
        {
            spdlog::info("name is : {}", s);
        }
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
    

    ///// onnx demo
    /// part-1
    // int run_mode = 1;
    // std::string engine_file, onnxModelpath = "../models/part1.onnx";
    // if (run_mode == 2) {
    //     engine_file = "int8-pointpillars-1.trt";
    // } else if (run_mode == 1) {
    //     engine_file = "fp16-pointpillars-1.trt";
    // } else {
    //     engine_file = "fp32-pointpillars-1.trt";
    // }
    // std::vector<std::string> customOutput{"pointpillars_part1/features"};
    // std::vector<float> input_data(4 * 8000 * 32, 1.0);

    /// part-2
    // int run_mode = 1;
    // std::string engine_file, onnxModelpath = "../models/part2.onnx";
    // if (run_mode == 2) {
    //     engine_file = "int8-pointpillars-2.trt";
    // } else if (run_mode == 1) {
    //     engine_file = "fp16-pointpillars-2.trt";
    // } else {
    //     engine_file = "fp32-pointpillars-2.trt";
    // }
    // std::vector<std::string> customOutput{"pointpillars_part2/features"};
    // std::vector<float> input_data(16 * 320 * 320, 1.0);

    // std::vector<std::vector<float>> output_data(customOutput.size());
    // Trt* onnx_net = new Trt();
    // onnx_net->CreateEngine(onnxModelpath, engine_file, customOutput, 1, run_mode);
    // for(int i = 0; i < 100; ++i) {
    //     trt_onnx(onnx_net, input_data, customOutput, output_data);
    // }
    // std::ofstream outfile("../trt-out-2-fp16.txt");
    // std::mutex coutMutex;
    // for(int i = 0; i < output_data[0].size(); ++i) {
    //     coutMutex.lock();
    //     outfile << output_data[0][i] << "\n";
    //     coutMutex.unlock();
    // }
    return 0;
}