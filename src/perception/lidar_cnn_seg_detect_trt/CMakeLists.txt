cmake_minimum_required(VERSION 3.10.2)
project(lidar_cnn_seg_detect_trt)
# add_compile_options(std=c++11)
set(CMAKE_CXX_FLAGS "-std=c++11 -O2 -Wfloat-conversion")

find_package(CUDA)
find_package(Eigen3)
find_package(Boost)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
add_definitions(${PCL_DEFINITIONS})
# link_directories(/usr/lib/x86_64-linux-gnu)


# link_directories(${CMAKE_SOURCE_DIR}/../lib)
set(TENSORRT_ROOT /opt/TensorRT-7.0.0.11)
# set(TENSORRT_INCLUDE_DIR ${TENSORRT_ROOT}/include)
set(TENSORRT_LIBRARIES_DIR ${TENSORRT_ROOT}/lib)
# include_directories(${TENSORRT_INCLUDE_DIR})
link_directories(${TENSORRT_LIBRARIES_DIR})

find_package(catkin REQUIRED COMPONENTS
        # autoware_build_flags
        roscpp
        tf
        tf_conversions
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
        tf
        tf_conversions
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
        ${ZZZ_ROOT}/devel/include
)
# target_compile_options(lidar_cnn_seg_detect_trt PUBLIC -std=c++11 -Wall -Wfloat-conversion)
add_executable(lidar_cnn_seg_detect_trt
        src/cluster2d.cpp
        src/feature_generator.cpp
        node/lidar_cnn_seg_detect_node.cpp
        node/cnn_segmentation.cpp
        )
target_link_libraries(lidar_cnn_seg_detect_trt
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${CUDA_LIBRARIES}
        # ${CUDA_CUBLAS_LIBRARIES}
        # ${CUDA_curand_LIBRARY}
        ${PCL_LIBRARIES}
        # glog
        trt
        nvinfer
        nvinfer_plugin
        nvparsers
        nvonnxparser
        nvcaffe_parser
        ${CUDART}
        )
add_dependencies(lidar_cnn_seg_detect_trt
        ${catkin_EXPORTED_TARGETS}
        )

install(TARGETS
        lidar_cnn_seg_detect_trt
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)

