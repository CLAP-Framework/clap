#pragma once 

#include <vector>
#include <stdlib.h>
#include <Eigen/Dense>

namespace nova {
    

struct Header {
    size_t      seq;
    timespec    timestamp;
};

struct Object {
    Header      header;
    float       score;
    float       yaw;
    size_t      id;
    int         label;
    Eigen::Vector3f pose;
    Eigen::Vector4f orie;
    Eigen::Vector3f dims;
    Eigen::Vector3f velo;
    Eigen::Vector3f acce;
}; // struct Object 

typedef std::vector<Object> ObjectArray;

} // namespace nova


