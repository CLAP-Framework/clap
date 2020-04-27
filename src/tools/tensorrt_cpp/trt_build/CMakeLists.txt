cmake_minimum_required(VERSION 3.0)
project(trt)
set(CMAKE_CXX_FLAGS "-std=c++11 -O2")

find_package(CUDA REQUIRED)

include(cmake/CUDA_utils.cmake)
# Set what architectures does nvcc support
set(CUDA_TARGET_ARCHS_SORTED ${CUDA_TARGET_ARCHS})
list(SORT CUDA_TARGET_ARCHS_SORTED)
CUDA_find_supported_arch_values(CUDA_targeted_archs ${CUDA_TARGET_ARCHS_SORTED})
message(STATUS "CUDA targeted archs: ${CUDA_targeted_archs}")
if (NOT CUDA_targeted_archs)
  message(FATAL_ERROR "None of the provided CUDA architectures ({${CUDA_TARGET_ARCHS}}) is supported by nvcc, use one or more of: ${CUDA_supported_archs}")
endif()
CUDA_get_gencode_args(CUDA_gencode_flags ${CUDA_targeted_archs})
message(STATUS "Generated gencode flags: ${CUDA_gencode_flags}")
# Add ptx & bin flags for cuda
set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} ${CUDA_gencode_flags}")

include_directories(./include)
include_directories(./spdlog)
include_directories(./)

# TensorRT
option(arm "The project will building on arm platform(TRT 5.1.6.1)." OFF)
if(arm)
  link_directories(/usr/local/lib)
  add_definitions(-DARM)
  message(STATUS "The project will building on arm platform(TRT 5.1.6.1).")
  set(TENSORRT_INCLUDE_DIR /usr/include/aarch64-linux-gnu/)
  set(TENSORRT_LIBRARIES_DIR /usr/lib/aarch64-linux-gnu/)
endif()

option(x86 "The project will building on x86 platform(TRT 7.0.0.11)." ON)
if(x86) 
  add_definitions(-DX86)
  message(STATUS "The project will building on x86 platform(TRT 7.0.0.11).")
  set(TENSORRT_ROOT /opt/TensorRT-7.0.0.11)
  set(TENSORRT_INCLUDE_DIR ${TENSORRT_ROOT}/include)
  set(TENSORRT_LIBRARIES_DIR ${TENSORRT_ROOT}/lib)
endif()

include_directories(${TENSORRT_INCLUDE_DIR})
link_directories(${TENSORRT_LIBRARIES_DIR})

file(GLOB_RECURSE trt_source
     Trt.cpp
     config.cpp
     )

cuda_add_library(trt SHARED ${trt_source})
target_compile_options(trt PUBLIC -std=c++11 -O2 -Wall -Wfloat-conversion)
set_target_properties(trt PROPERTIES POSITION_INDEPENDENT_CODE ON)

# file(GLOB_RECURSE test_source
#      builder.cpp
#      )
#  add_executable(trt_builder ${test_source})
#  target_compile_options(trt_builder PUBLIC -std=c++11 -Wall -Wfloat-conversion)
#  target_link_libraries(trt_builder trt yaml-cpp)
#  target_link_libraries(trt_builder nvinfer)
#  target_link_libraries(trt_builder nvinfer_plugin)
#  target_link_libraries(trt_builder nvparsers)
#  target_link_libraries(trt_builder nvonnxparser)
#  target_link_libraries(trt_builder nvcaffe_parser)
#  target_link_libraries(trt_builder ${CUDART})
