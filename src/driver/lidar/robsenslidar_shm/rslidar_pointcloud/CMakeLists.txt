cmake_minimum_required(VERSION 2.8.3)
project(rslidar_pointcloud)

add_compile_options(-std=c++11 -O2)
set(CMAKE_BUILD_TYPE Release)#RelWithDebInfo
set(${PROJECT_NAME}_CATKIN_DEPS
    angles
    nodelet
    pcl_ros
    roscpp
    roslib
    sensor_msgs
    tf
    rslidar_msgs
    dynamic_reconfigure
    diagnostic_updater
)
set(libpcap_LIBRARIES -lpcap)

find_package(catkin REQUIRED COMPONENTS
             ${${PROJECT_NAME}_CATKIN_DEPS} 
       pcl_conversions
       rospy
       std_msgs
       genmsg
       cv_bridge
       message_generation)

find_package(Boost REQUIRED COMPONENTS thread)
find_package(PkgConfig REQUIRED)
include_directories(${catkin_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
  include
)

generate_dynamic_reconfigure_options(cfg/CloudNode.cfg
                                                                                    cfg/rslidarNode.cfg )
#generate_dynamic_reconfigure_options(cfg/rslidarNode.cfg)     #TODO need to make the two files one?

catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS}
        CATKIN_DEPENDS message_runtime std_msgs     
)

add_library(rslidar_data src/rawdata.cpp)
target_link_libraries(rslidar_data
    ${catkin_LIBRARIES}
    ${libpcap_LIBRARIES})

add_library(rslidar_input src/input.cpp)
    target_link_libraries(rslidar_input
      ${catkin_LIBRARIES}
      ${libpcap_LIBRARIES})
add_dependencies(rslidar_input rslidar_msgs_gencpp)

add_library(rslidar_shm_object src/shm_object.cpp)
    target_link_libraries(rslidar_shm_object
      ${catkin_LIBRARIES})

add_library(rslidar_sem_object src/sem_object.cpp)
    target_link_libraries(rslidar_sem_object
      ${catkin_LIBRARIES})
      
add_library(rslidar_point_struct src/point_struct.cpp)
    target_link_libraries(rslidar_point_struct
      ${catkin_LIBRARIES})      

add_library(rslidar_point src/convert.cpp)
    target_link_libraries(rslidar_point
      rslidar_data
      rslidar_input
      rslidar_shm_object
      rslidar_sem_object
      rslidar_point_struct
      ${catkin_LIBRARIES})
        
add_library(cloud_nodelet src/cloud_nodelet.cpp)
target_link_libraries(cloud_nodelet 
                        rslidar_point
                      ${catkin_LIBRARIES})

add_executable(cloud_node src/cloud_node.cpp)

if(catkin_EXPORTED_TARGETS)
  add_dependencies(rslidar_data 
                                           ${catkin_EXPORTED_TARGETS})
endif()

target_link_libraries(cloud_node
    rslidar_point
    rslidar_input
    ${catkin_LIBRARIES}
    ${libpcap_LIBRARIES})

