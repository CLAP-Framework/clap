cmake_minimum_required(VERSION 3.10.2)
project(lidar_pointpillars_detect_trt)
set(CMAKE_CXX_FLAGS "-std=c++11 -O2 -Wfloat-conversion")

option(USE_GPU "use gpu" ON)
if (USE_GPU)
message("-- USE_GPU")
add_definitions(-D__USE_GPU__)
endif ()

find_package(CUDA)
find_package(Boost)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
add_definitions(${PCL_DEFINITIONS})

include(FindCUDA)
set(CUDA_ARCH_LIST Auto CACHE STRING
    "List of CUDA architectures (e.g. Pascal, Volta, etc) or \
compute capability versions (6.1, 7.0, etc) to generate code for. \
Set to Auto for automatic detection (default)."
)
cuda_select_nvcc_arch_flags(CUDA_ARCH_FLAGS ${CUDA_ARCH_LIST})
list(APPEND CUDA_NVCC_FLAGS ${CUDA_ARCH_FLAGS})

message("-- system processor: ${CMAKE_SYSTEM_PROCESSOR}")
if ("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "x86_64")
  add_definitions(-DX86)
  message(STATUS "The project will building on x86 platform(TRT 7.0.0.11).")
  include_directories("/usr/local/cuda-10.0/include")
  set(TENSORRT_ROOT /opt/TensorRT-7.0.0.11)
  set(TENSORRT_LIBRARIES_DIR ${TENSORRT_ROOT}/lib)
else ()
  MESSAGE(STATUS "The project will built on arm platform(TRT 5.1.6.1).") 
  link_directories(/usr/local/lib)
  include_directories("/usr/local/cuda-10.0/targets/aarch64-linux/include/")
  add_definitions(-DARM)
  set(TENSORRT_INCLUDE_DIR /usr/include/aarch64-linux-gnu/)
  set(TENSORRT_LIBRARIES_DIR /usr/lib/aarch64-linux-gnu/)
  include_directories(${TENSORRT_INCLUDE_DIR})
endif ()
link_directories(${TENSORRT_LIBRARIES_DIR})

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  sensor_msgs
  geometry_msgs
  autoware_msgs
)

catkin_package(
  INCLUDE_DIRS
  include
  CATKIN_DEPENDS
  roscpp
  pcl_ros
  sensor_msgs
  geometry_msgs
  autoware_msgs
)

include_directories(
  include
  ${CMAKE_SOURCE_DIR}/tools/tensorrt_cpp/trt_infer
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

if (USE_GPU)
    cuda_add_library(gpu_postprocess_lib SHARED
      src/postprocess.cu)
    target_compile_options(gpu_postprocess_lib PUBLIC -std=c++11 -O2 -Wall -Wfloat-conversion)
    set_target_properties(gpu_postprocess_lib PROPERTIES POSITION_INDEPENDENT_CODE ON)

    set(SOURCES_FILE
        node/lidar_pointpillars_detect_node.cpp
        node/pointpillars.cpp
        src/postprocess.cpp
        src/feature_generator.cpp
        )
    add_executable(lidar_pointpillars_detect_trt
                  ${SOURCES_FILE}
                  )
    target_link_libraries(lidar_pointpillars_detect_trt
                  ${catkin_LIBRARIES}
                  ${OpenCV_LIBRARIES}
                  ${CUDA_LIBRARIES}
                  ${PCL_LIBRARIES}
                  gpu_postprocess_lib
                  trt
                  nvinfer
                  nvinfer_plugin
                  nvparsers
                  nvonnxparser
                  nvcaffe_parser
                  ${CUDART}
                )

    add_dependencies(lidar_pointpillars_detect_trt
                  ${catkin_EXPORTED_TARGETS}
                    )
else ()
    set(SOURCES_FILE
    node/lidar_pointpillars_detect_node.cpp
    node/pointpillars.cpp
    src/postprocess.cpp
    src/feature_generator.cpp
    )
    add_executable(lidar_pointpillars_detect_trt
              ${SOURCES_FILE}
              )
    target_link_libraries(lidar_pointpillars_detect_trt
              ${catkin_LIBRARIES}
              ${OpenCV_LIBRARIES}
              ${CUDA_LIBRARIES}
              ${PCL_LIBRARIES}
              trt
              nvinfer
              nvinfer_plugin
              nvparsers
              nvonnxparser
              nvcaffe_parser
              ${CUDART}
            )

    add_dependencies(lidar_pointpillars_detect_trt
              ${catkin_EXPORTED_TARGETS}
                )
endif()

if (USE_GPU)
    install(TARGETS
        gpu_postprocess_lib
        lidar_pointpillars_detect_trt
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
else ()
    install(TARGETS
        lidar_pointpillars_detect_trt
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
endif ()

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)

