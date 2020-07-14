#ifndef INFERENCE_FACTORY_H___
#define INFERENCE_FACTORY_H___

#include <string>

#include "inference.h"

namespace novauto {
namespace tensorrt {
namespace inference {

Inference *CreateInferenceByName(const std::string &name,
                                 const std::string &net_file,
                                 const std::vector<std::string> &inputs,
                                 const std::vector<std::string> &outputs);

}  // namespace inference
}  // namespace tensorrt
}  // namespace novauto

#endif 