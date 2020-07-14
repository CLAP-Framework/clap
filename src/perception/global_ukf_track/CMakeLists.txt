cmake_minimum_required(VERSION 2.8.3)
project(global_ukf_track)

add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)
find_package(autoware_msgs REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  geometry_msgs
  tf
#   vector_map
  autoware_msgs
  )

find_package(Boost REQUIRED COMPONENTS system filesystem)


set(CMAKE_CXX_FLAGS "-O2 -Wall ${CMAKE_CXX_FLAGS}")

catkin_package(
  CATKIN_DEPENDS
  roscpp
  pcl_ros
  autoware_msgs
  tf
#   vector_map
  )

include_directories(
  ${autoware_msgs_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  include
  src/ukf
  ${Eigen3_INCLUDE_DIR}
  ${ZZZ_ROOT}/devel/include
)

#imm_ukf_pda
add_executable(global_ukf_track
  src/main.cc
  src/ukf/ukf.cpp
  src/ukf/tools.cpp
  src/lidar_track_ros.cc
  src/lidar_track.cc
  src/HungarianAlg.cpp
)
target_link_libraries(global_ukf_track
  ${catkin_LIBRARIES} yaml-cpp  
  ${Boost_LIBRARIES}
)

add_dependencies(global_ukf_track
  ${catkin_EXPORTED_TARGETS}
  )


install(TARGETS
global_ukf_track
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)
