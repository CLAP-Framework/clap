cmake_minimum_required(VERSION 3.0)
project(trt)
set(CMAKE_CXX_FLAGS "-std=c++11 -O2")
option(BUILD_TEST "compile test" ON)

find_package(CUDA REQUIRED)

include_directories(./include)
include_directories(./third/spdlog)
include_directories(./third/)
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
     inference_factory.cc
     inference.cc
     trt_net.cc
     )
cuda_add_library(trt SHARED ${trt_source})
target_compile_options(trt PUBLIC -std=c++11 -O2 -Wall -Wfloat-conversion)
set_target_properties(trt PROPERTIES POSITION_INDEPENDENT_CODE ON)

# custom test
if(BUILD_TEST)
  message(STATUS "Build test")
  file(GLOB_RECURSE test_source
      test.cpp
      config.cc
      )
  add_executable(unit_test ${test_source})
  target_compile_options(unit_test PUBLIC -std=c++11 -O2 -Wall -Wfloat-conversion)
  target_link_libraries(unit_test trt yaml-cpp)
  target_link_libraries(unit_test nvinfer)
  target_link_libraries(unit_test nvinfer_plugin)
  target_link_libraries(unit_test nvparsers)
  target_link_libraries(unit_test nvonnxparser)
  target_link_libraries(unit_test nvcaffe_parser)
  target_link_libraries(unit_test ${CUDART})
endif()
