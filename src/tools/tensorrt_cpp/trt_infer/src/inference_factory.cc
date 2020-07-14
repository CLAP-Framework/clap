
#include "inference_factory.h"
#include "trt_net.h"

namespace novauto {
namespace tensorrt {
namespace inference {

Inference *CreateInferenceByName(const std::string &name,
                                 const std::string &net_file,
                                 const std::vector<std::string> &inputs,
                                 const std::vector<std::string> &outputs) {
    if (name == "TRTNet") {
        return new TRTNet(net_file);
    } else if (name == "TRTNetFP16") {
        // return new RTNet(proto_file, weight_file, outputs, inputs, model_root);
    } else if (name == "TRTNetINT8") {
        // return new PaddleNet(proto_file, weight_file, outputs, inputs);
    }
    return nullptr;
}

}  // namespace inference
}  // namespace tensorrt
}  // namespace novauto
