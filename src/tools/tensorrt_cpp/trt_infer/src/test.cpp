#include "inference_factory.h"
#include "spdlog/spdlog.h"
#include "config.h"

#include <iostream>
#include <memory>

int main(int argc, char *argv[]) {
    if (argc < 2) {
        spdlog::error("Please input the configuration file path.");
        return 0;
    }
    loadYamlFile(argv[1]);
    std::vector<float> input_data(gChannel * gHeight * gWidth, 1.0);
    std::vector<std::vector<float> >output_data(gOutputNode.size());
    std::shared_ptr<novauto::tensorrt::inference::Inference> inference;
    inference.reset(novauto::tensorrt::inference::CreateInferenceByName(
                    "TRTNet",\
                    gEngineName,\
                    gInputNode, gOutputNode));
    for (int i = 0; i < 1000; ++i) {
        for(int j = 0; j < gInputNode.size(); ++j) {
            inference -> SetInputTensor(j, input_data);
        }
        inference -> Infer();
        for(int j = 0; j < gOutputNode.size(); ++j) {
            inference -> GetOutputTensor(j + gInputNode.size(), output_data[j]);
        }
    }
    return 0;
}