cmake_minimum_required(VERSION 2.8.3)
project(global_localization)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  geodesy
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  rosbag
  std_msgs
  image_transport
  cv_bridge
  tf
  novatel_gnss_msgs
  gps_common
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)

include_directories(
  # /home/jiangxx/other_ws/localization/devel/include
  ${catkin_INCLUDE_DIRS} 
  include
)

message("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++path" ${catkin_INCLUDE_DIRS} )

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs 
  DEPENDS EIGEN3 PCL 
  INCLUDE_DIRS
)


#  oxford_node
add_executable(global_fusion_oxford_node 
	src/global_fusion_oxford.cpp 
)
target_link_libraries(global_fusion_oxford_node ${catkin_LIBRARIES} 
  ${Boost_LIBRARIES} ) 
add_dependencies(global_fusion_oxford_node novatel_gnss_msgs_gencpp) 

#  oxford_node  修改坐标系 前x,左y,上zs
add_executable(global_fusion_oxford_node2 
	src/global_fusion_oxford_2.cpp 
)
target_link_libraries(global_fusion_oxford_node2 ${catkin_LIBRARIES}
  ${Boost_LIBRARIES} ) 
add_dependencies(global_fusion_oxford_node2 novatel_gnss_msgs_gencpp) 


#  m2_node
add_executable(global_fusion_m2_node 
	src/global_fusion_m2.cpp 
)
target_link_libraries(global_fusion_m2_node ${catkin_LIBRARIES} 
  ${Boost_LIBRARIES} ) 
add_dependencies(global_fusion_m2_node novatel_gnss_msgs_gencpp) 

#  m2_node
add_executable(global_fusion_m2_node2 
	src/global_fusion_m2_2.cpp 
)
target_link_libraries(global_fusion_m2_node2 ${catkin_LIBRARIES}
  ${Boost_LIBRARIES} ) 
add_dependencies(global_fusion_m2_node2 novatel_gnss_msgs_gencpp) 



