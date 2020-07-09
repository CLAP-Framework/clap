cmake_minimum_required(VERSION 3.0)
project(trt)
set(CMAKE_CXX_FLAGS "-std=c++11 -O2")

find_package(CUDA REQUIRED)


include_directories(./include)
include_directories(./spdlog)
include_directories(./)

# TensorRT
MESSAGE(STATUS "The platform arch is " ${CMAKE_HOST_SYSTEM_PROCESSOR} ".")
IF (CMAKE_SYSTEM_NAME MATCHES "x86_64") 
  MESSAGE(STATUS "The project will built on x86 platform(TRT 7.0.0.11).")
  add_definitions(-DX86)
  set(TENSORRT_ROOT /opt/TensorRT-7.0.0.11)
  set(TENSORRT_INCLUDE_DIR ${TENSORRT_ROOT}/include)
  set(TENSORRT_LIBRARIES_DIR ${TENSORRT_ROOT}/lib)
  include_directories(${TENSORRT_INCLUDE_DIR})
  link_directories(${TENSORRT_LIBRARIES_DIR})
ELSE ()
  MESSAGE(STATUS "The project will built on arm platform(TRT 5.1.6.1).") 
  link_directories(/usr/local/lib)
  add_definitions(-DARM)
  set(TENSORRT_INCLUDE_DIR /usr/include/aarch64-linux-gnu/)
  set(TENSORRT_LIBRARIES_DIR /usr/lib/aarch64-linux-gnu/)
  include_directories(${TENSORRT_INCLUDE_DIR})
  link_directories(${TENSORRT_LIBRARIES_DIR})
ENDIF()

file(GLOB_RECURSE trt_source
     Trt.cpp
     config.cpp
     )
cuda_add_library(trt SHARED ${trt_source})
target_compile_options(trt PUBLIC -std=c++11 -O2 -Wall -Wfloat-conversion)
set_target_properties(trt PROPERTIES POSITION_INDEPENDENT_CODE ON)

 file(GLOB_RECURSE test_source
      builder.cpp
      )
  add_executable(trt_builder ${test_source})
  target_compile_options(trt_builder PUBLIC -std=c++11 -Wall -Wfloat-conversion)
  target_link_libraries(trt_builder trt yaml-cpp)
  target_link_libraries(trt_builder nvinfer)
  target_link_libraries(trt_builder nvinfer_plugin)
  target_link_libraries(trt_builder nvparsers)
  target_link_libraries(trt_builder nvonnxparser)
  target_link_libraries(trt_builder nvcaffe_parser)
  target_link_libraries(trt_builder ${CUDART})
