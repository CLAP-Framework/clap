cmake_minimum_required(VERSION 2.8.3)
project(trt)

add_compile_options(-std=c++11 -O2)
set(TRT_INFER_ROOT "${CMAKE_SOURCE_DIR}/tools/tensorrt_cpp/trt_infer")
set(TRT_BUILDER_ROOT "${CMAKE_SOURCE_DIR}/tools/tensorrt_cpp/trt_build")

message("-- system processor: ${CMAKE_SYSTEM_PROCESSOR}")
if ("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "x86_64")
  add_definitions(-DX86)
  message(STATUS "The project will building on x86 platform(TRT 7.0.0.11).")
  set(TENSORRT_ROOT /opt/TensorRT-7.0.0.11)
  set(TENSORRT_INCLUDE_DIR ${TENSORRT_ROOT}/include)
  set(TENSORRT_LIBRARIES_DIR ${TENSORRT_ROOT}/lib)
else ()
  link_directories(/usr/local/lib)
  add_definitions(-DARM)
  message(STATUS "The project will building on arm platform(TRT 5.1.6.1).")
  set(TENSORRT_INCLUDE_DIR /usr/include/aarch64-linux-gnu/)
  set(TENSORRT_LIBRARIES_DIR /usr/lib/aarch64-linux-gnu/)
endif ()

include_directories(${TENSORRT_INCLUDE_DIR})
link_directories(${TENSORRT_LIBRARIES_DIR})
find_package(catkin REQUIRED COMPONENTS
  roscpp
)
catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS
  roscpp
)

option(BUILD_TRT_BUILDER "compile builder" ON)
if(BUILD_TRT_BUILDER)

  find_package(CUDA REQUIRED)
  include(${TRT_BUILDER_ROOT}/cmake/CUDA_utils.cmake)
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

  include_directories(${TRT_BUILDER_ROOT}/include)
  include_directories(${TRT_BUILDER_ROOT}/spdlog)
  include_directories(${TRT_BUILDER_ROOT})

  file(GLOB_RECURSE trt_builder_lib_source
    ${TRT_BUILDER_ROOT}/src/Trt.cpp
    ${TRT_BUILDER_ROOT}/src/config.cpp
  )
  cuda_add_library(trt_builder_lib SHARED ${trt_builder_lib_source})
  target_compile_options(trt_builder_lib PUBLIC -std=c++11 -O2 -Wall -Wfloat-conversion)
  set_target_properties(trt_builder_lib PROPERTIES POSITION_INDEPENDENT_CODE ON)

  file(GLOB_RECURSE builder_source
    ${TRT_BUILDER_ROOT}/builder.cpp
  )
  add_executable(trt_builder ${builder_source})
  target_compile_options(trt_builder PUBLIC -std=c++11 -Wall -Wfloat-conversion)
  target_link_libraries(trt_builder trt_builder_lib yaml-cpp)
  target_link_libraries(trt_builder nvinfer)
  target_link_libraries(trt_builder nvinfer_plugin)
  target_link_libraries(trt_builder nvparsers)
  target_link_libraries(trt_builder nvonnxparser)
  target_link_libraries(trt_builder nvcaffe_parser)
  target_link_libraries(trt_builder ${CUDART})
  
  target_link_libraries(trt_builder
    ${catkin_LIBRARIES}
  )

  add_dependencies(trt_builder
    ${catkin_EXPORTED_TARGETS}
  )

endif()


option(BUILD_TRT_INFER "compile infer" ON)
if(BUILD_TRT_INFER)


  option(BUILD_TEST "compile test" OFF)
  find_package(CUDA REQUIRED)
  include(${TRT_INFER_ROOT}/cmake/CUDA_utils.cmake)
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

  include_directories(${TRT_INFER_ROOT}/include)
  include_directories(${TRT_INFER_ROOT}/third/spdlog)
  include_directories(${TRT_INFER_ROOT}/third/)
  include_directories(${TRT_INFER_ROOT})

  file(GLOB_RECURSE trt_source
    ${TRT_INFER_ROOT}/src/inference_factory.cc
    ${TRT_INFER_ROOT}/src/inference.cc
    ${TRT_INFER_ROOT}/src/trt_net.cc
  )
  cuda_add_library(trt SHARED ${trt_source})
  target_compile_options(trt PUBLIC -std=c++11 -O2 -Wall -Wfloat-conversion)
  set_target_properties(trt PROPERTIES POSITION_INDEPENDENT_CODE ON)

  # custom test
  # if(BUILD_TEST)
  #   message(STATUS "Build test")
  #   file(GLOB_RECURSE test_source
  #     ${TRT_INFER_ROOT}/src/config.cc
  #     ${TRT_INFER_ROOT}/src/test.cpp
  #   )
  #   add_executable(unit_test ${test_source})
  #   target_compile_options(unit_test PUBLIC -std=c++11 -Wall -Wfloat-conversion)
  #   target_link_libraries(unit_test trt yaml-cpp)
  #   target_link_libraries(unit_test nvinfer)
  #   target_link_libraries(unit_test nvinfer_plugin)
  #   target_link_libraries(unit_test nvparsers)
  #   target_link_libraries(unit_test nvonnxparser)
  #   target_link_libraries(unit_test nvcaffe_parser)
  #   target_link_libraries(unit_test ${CUDART})
  # endif()

endif()


install(TARGETS trt_builder trt
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install(DIRECTORY ${TRT_INFER_ROOT}
# 	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# 	FILES_MATCHING PATTERN "*.h"
# )

# install(DIRECTORY launch/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
#   PATTERN ".svn" EXCLUDE
# )

