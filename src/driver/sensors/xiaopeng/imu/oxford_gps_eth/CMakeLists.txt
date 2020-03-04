cmake_minimum_required(VERSION 2.8.3)
project(oxford_gps_eth)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_gps_node src/node.cpp)
target_link_libraries(${PROJECT_NAME}_gps_node ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME}_gps_node PROPERTIES OUTPUT_NAME gps_node PREFIX "")

install(TARGETS ${PROJECT_NAME}_gps_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
  add_subdirectory(tests)
endif()
