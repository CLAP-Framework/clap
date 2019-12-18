cmake_minimum_required(VERSION 2.8.3)
project(zzz_perception_detection_lidar_detectors)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  zzz_perception_msgs
  pcl_ros
)

catkin_package(
  CATKIN_DEPENDS sensor_msgs zzz_perception_msgs pcl_ros
  # DEPENDS Boost
  INCLUDE_DIRS include
  LIBRARIES zzz_perception_detection_lidar_detectors
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Add library target
add_library(zzz_perception_detection_lidar_detectors
  src/LidarDetector.cpp
  src/EuclideanClusterDetector.cpp
)
add_dependencies(zzz_perception_detection_lidar_detectors
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(zzz_perception_detection_lidar_detectors
  ${catkin_LIBRARIES}
)

# Add node targets
add_executable(euclidean_cluster_detector nodes/euclidean_cluster_detector.cpp)
set_target_properties(euclidean_cluster_detector PROPERTIES OUTPUT_NAME euclidean_cluster PREFIX "")
add_dependencies(euclidean_cluster_detector
  zzz_perception_detection_lidar_detectors
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(euclidean_cluster_detector
  zzz_perception_detection_lidar_detectors
  ${catkin_LIBRARIES}
)

# Install files
install(TARGETS euclidean_cluster_detector
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
