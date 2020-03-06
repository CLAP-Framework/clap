cmake_minimum_required(VERSION 2.8.3)
project(oxford_gps_eth)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf
  message_generation
  std_msgs
)
## Generate messages in the 'msg' folder
add_message_files(DIRECTORY msg FILES
RigidBodyState.msg
RigidBodyStateStamped.msg
  
)
## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  geometry_msgs
)
catkin_package(
)





catkin_package(
  CATKIN_DEPENDS
  message_runtime
  std_msgs
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
