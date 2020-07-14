#ifndef CONFIG_H__
#define CONFIG_H__

#include <iostream>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>
#include "spdlog/spdlog.h"

void loadYamlFile(const std::string &name);

extern std::string gModelType;
extern int gDataType;
extern int gMaxBatchSize;
extern int gMaxWorkspaceSize;
extern std::string gEngineName;
extern std::vector<std::string> gInputNode;
extern std::vector<std::string> gOutputNode;

//// caffe model
extern std::string gCaffePrototxt;
extern std::string gCaffeModel;
extern bool gCnnSeg;
//// onnx model
extern std::string gOnnxModel;
//// uff model
extern std::string gUffModel;

template <typename T>
inline void operator>>(const YAML::Node& node, T& i) {
    i = node.as<T>();
}
// 读取单个变量的值
template <typename T>
inline void GetParameter(YAML::Node node, std::string name, T &i) {
    try {
        node[name] >> i;
    } catch (std::exception e) {
        spdlog::error("There is no Parma {} .", name);
        exit(0);
    }
}
// 读取数组变量的值
template <typename T>
inline void GetParameter(YAML::Node node, std::string name,
                  T *i, int idx) {
    try {
        node[name][idx] >> i[idx];
    } catch (std::exception e) {
        spdlog::error("There is no Parma {} .", name);
        exit(0);
    }
}

#endif // CONFIG_H__