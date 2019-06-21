cmake_minimum_required(VERSION 2.8.3)
project(zzz_perception_detection_lidar_detectors)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  zzz_perception_msgs
  sensor_msgs
  pcl_ros
  pcl_conversions
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(regiongrow_detector nodes/regiongrow_detector.cpp)
set_target_properties(regiongrow_detector PROPERTIES OUTPUT_NAME regiongrow PREFIX "")

add_dependencies(regiongrow_detector ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(regiongrow_detector
  ${catkin_LIBRARIES}
)

install(TARGETS regiongrow_detector
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

