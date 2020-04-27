cmake_minimum_required(VERSION 3.0)
project(trt)
set(CMAKE_CXX_FLAGS "-std=c++11 -O2")

option(BUILD_TEST "compile test" ON)

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
include_directories(./third/spdlog)
include_directories(./third/)
include_directories(./)

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
      config.cc )
      
  add_executable(unit_test ${test_source})
  target_compile_options(unit_test PUBLIC -std=c++11 -Wall -Wfloat-conversion)
  target_link_libraries(unit_test trt yaml-cpp)
  target_link_libraries(unit_test nvinfer)
  target_link_libraries(unit_test nvinfer_plugin)
  target_link_libraries(unit_test nvparsers)
  target_link_libraries(unit_test nvonnxparser)
  target_link_libraries(unit_test nvcaffe_parser)
  target_link_libraries(unit_test ${CUDART})
endif()
