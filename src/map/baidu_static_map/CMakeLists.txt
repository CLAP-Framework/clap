cmake_minimum_required(VERSION 2.8.3)
project(baidu_static_map)

# add_compile_options(std=c++11)
# add_compile_options(-O2)

find_package(catkin REQUIRED 
  roscpp 
  zzz_driver_msgs 
  std_msgs 
  sensor_msgs 
  zzz_navigation_msgs
  geometry_msgs
  nav_msgs
)

catkin_package()

set (HDMAP_SDK_PATH ${CMAKE_SOURCE_DIR}/../install)

link_directories(
  ${HDMAP_SDK_PATH}/lib
)
find_package(Eigen3 REQUIRED)
find_package(Protobuf REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Protobuf_INCLUDE_DIRS}
  ${HDMAP_SDK_PATH}/include
)

add_executable(baidu_static_map 
  src/main.cc
)
target_link_libraries(baidu_static_map
  ${catkin_LIBRARIES}
  ${Protobuf_LIBRARIES}
  hdmap
  proj
  tinyxml2
  glog
  gflags
  yaml-cpp
)
add_dependencies(baidu_static_map
  ${catkin_EXPORTED_TARGETS}
)

install(TARGETS
  baidu_static_map
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
  PATTERN ".git" EXCLUDE
)
