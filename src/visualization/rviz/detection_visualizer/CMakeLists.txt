cmake_minimum_required(VERSION 2.8.3)
project(zzz_visualization_rviz_detection_visualizer)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  visualization_msgs
  zzz_perception_msgs
)

catkin_package(
  CATKIN_DEPENDS visualization_msgs zzz_perception_msgs
  INCLUDE_DIRS include
  LIBRARIES zzz_visualization_rviz_detection_visualizer
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Add library target
add_library(zzz_visualization_rviz_detection_visualizer
  src/BBoxVisualizer.cpp
)
add_dependencies(zzz_visualization_rviz_detection_visualizer
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(zzz_visualization_rviz_detection_visualizer
  ${catkin_LIBRARIES}
)

# Add node targets
add_executable(bbox_visualizer nodes/bbox_visualizer.cpp)
set_target_properties(bbox_visualizer PROPERTIES OUTPUT_NAME bbox PREFIX "")

add_dependencies(bbox_visualizer
  zzz_visualization_rviz_detection_visualizer
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(bbox_visualizer
  zzz_visualization_rviz_detection_visualizer
  ${catkin_LIBRARIES}
)

# Install files
install(TARGETS bbox_visualizer
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
