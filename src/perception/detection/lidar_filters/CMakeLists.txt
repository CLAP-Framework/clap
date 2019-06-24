cmake_minimum_required(VERSION 2.8.3)
project(zzz_perception_detection_lidar_filters)

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
  LIBRARIES zzz_perception_detection_lidar_filters
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Add library target
add_library(zzz_perception_detection_lidar_filters
  src/LShapeFilter.cpp
)

# Add node targets
add_executable(lshape_filter nodes/lshape_filter.cpp)
set_target_properties(lshape_filter PROPERTIES OUTPUT_NAME lshape PREFIX "")
add_dependencies(lshape_filter
  zzz_perception_detection_lidar_filters
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(lshape_filter
  zzz_perception_detection_lidar_filters
  ${catkin_LIBRARIES}
)

# Install files
install(TARGETS lshape_filter
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
