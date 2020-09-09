cmake_minimum_required(VERSION 3.10.2)
project(lidar_cnn_seg_detect_trt)
# add_compile_options(std=c++11)
set(CMAKE_CXX_FLAGS "-std=c++11 -O2 -Wfloat-conversion")

option(USE_GPU "use gpu" ON)

if (USE_GPU)
message("-- USE_GPU")
add_definitions(-D__USE_GPU__)
endif ()

find_package(CUDA)
find_package(Eigen3)
find_package(Boost)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
add_definitions(${PCL_DEFINITIONS})
# link_directories(/usr/lib/x86_64-linux-gnu)

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
  # add_definitions(-DX86)
  message(STATUS "The project will building on x86 platform(TRT 7.0.0.11).")
  set(TENSORRT_ROOT /opt/TensorRT-7.0.0.11)
  set(TENSORRT_LIBRARIES_DIR ${TENSORRT_ROOT}/lib)
else ()
  link_directories(/usr/local/lib)
  # add_definitions(-DARM)
  message(STATUS "The project will building on arm platform(TRT 5.1.6.1).")
  set(TENSORRT_LIBRARIES_DIR /usr/lib/aarch64-linux-gnu/)
endif ()

link_directories(${TENSORRT_LIBRARIES_DIR})

find_package(catkin REQUIRED COMPONENTS
  # autoware_build_flags
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

# include_directories(${CUDA_INCLUDE_DIRS})

include_directories(
  include
  ${CMAKE_SOURCE_DIR}/tools/tensorrt_cpp/trt_infer
  ${catkin_INCLUDE_DIRS}
  # ${Eigen_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

if (USE_GPU)
cuda_add_library(feature_gen SHARED 
  src/feature_generator_cuda.cu
  src/prekernel.cu
)
target_compile_options(feature_gen PUBLIC -std=c++11 -O2 -Wall -Wfloat-conversion)
set_target_properties(feature_gen PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_executable(lidar_cnn_seg_detect_trt
  src/cluster2d.cpp
  # src/feature_generator.cpp
  node/lidar_cnn_seg_detect_node.cpp
  node/cnn_segmentation.cpp
  src/shm_object.cpp
  src/sem_object.cpp
  src/point_struct.cpp
)

target_link_libraries(lidar_cnn_seg_detect_trt
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${CUDA_LIBRARIES}
  ${PCL_LIBRARIES}
  feature_gen
  trt
  nvinfer
  nvinfer_plugin
  nvparsers
  nvonnxparser
  nvcaffe_parser
  ${CUDART}
)
else ()

add_executable(lidar_cnn_seg_detect_trt
  src/cluster2d.cpp
  src/feature_generator.cpp
  node/lidar_cnn_seg_detect_node.cpp
  node/cnn_segmentation.cpp
  src/shm_object.cpp
  src/sem_object.cpp
  src/point_struct.cpp
)

target_link_libraries(lidar_cnn_seg_detect_trt
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
endif ()

add_dependencies(lidar_cnn_seg_detect_trt
  ${catkin_EXPORTED_TARGETS}
)

if (USE_GPU)
install(TARGETS
  lidar_cnn_seg_detect_trt feature_gen
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
else ()
install(TARGETS
  lidar_cnn_seg_detect_trt 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
endif ()


install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)

