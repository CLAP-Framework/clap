#include "config.h"

int gChannel;
int gHeight;
int gWidth;
std::string gEngineName;
std::vector<std::string> gInputNode;
std::vector<std::string> gOutputNode;

void loadYamlFile(const std::string &name) {
    YAML::Node node;
    try {
        node = YAML::LoadFile(name);
    } catch (std::exception e) {
        spdlog::error("Can't find configure file {} .", name);
    }
    // load parameter gDisplayIp
    GetParameter(node, "gChannel", gChannel);
    GetParameter(node, "gHeight", gHeight);
    GetParameter(node, "gWidth", gWidth);
    GetParameter(node, "gEngineName", gEngineName);
    GetParameter(node, "gInputNode", gInputNode);
    GetParameter(node, "gOutputNode", gOutputNode);
    spdlog::info("The {} model will be loaded...", gEngineName);
}




