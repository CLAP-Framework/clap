cmake_minimum_required(VERSION 2.8.3)
project(baidu_static_map)

set(CMAKE_CXX_FLAGS "-std=c++11 -fPIC ${CMAKE_CXX_FLAGS}")
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(catkin REQUIRED 
  roscpp 
  std_msgs 
  sensor_msgs 
  zzz_driver_msgs
  zzz_navigation_msgs
  geometry_msgs
  nav_msgs
)

find_package(Protobuf REQUIRED)
find_package(hdmap REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Qt5 REQUIRED Core Widgets)

message(${Qt5Core_INCLUDE_DIRS})

catkin_package()

link_directories(
  ${HDMAP_SDK_PATH}/lib
) 

find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Protobuf_INCLUDE_DIRS}
  ${hdmap_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
)

message("--------------------------------------")
message(${hdmap_INCLUDE_DIRS})

file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/*.hpp *.h)
file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
QT5_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT5_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

add_executable(fixed_path 
  node/fixed_path.cpp
  src/hdmapEngine.cpp
  src/hdmapConcept.cpp
)
target_link_libraries(fixed_path 
  ${catkin_LIBRARIES}
  ${Protobuf_LIBRARIES}
  ${OpenCV_LIBS}
  hdmap
  ${hdmap_LIBRARIES}
  proj
  tinyxml2
  glog
  gflags
  yaml-cpp
)
add_dependencies(fixed_path 
  ${catkin_EXPORTED_TARGETS}
)

add_executable(user_path 
  node/user_path.cpp
  src/hdmapEngine.cpp
  src/hdmapConcept.cpp
  src/routeSelectingWindow.cpp
  ${QT_FORMS_HPP}
  ${QT_MOC_HPP}
)
target_link_libraries(user_path 
  ${catkin_LIBRARIES}
  ${Protobuf_LIBRARIES}
  ${OpenCV_LIBS}
  hdmap
  ${hdmap_LIBRARIES}
  proj
  tinyxml2
  glog
  gflags
  yaml-cpp
  Qt5::Widgets
  Qt5::Core
)
add_dependencies(user_path 
  ${catkin_EXPORTED_TARGETS}
)

add_executable(export_path 
  node/export_path.cpp
  src/hdmapEngine.cpp
  src/hdmapConcept.cpp
)
target_link_libraries(export_path 
  ${catkin_LIBRARIES}
  ${Protobuf_LIBRARIES}
  ${OpenCV_LIBS}
  hdmap
  ${hdmap_LIBRARIES}
  proj
  tinyxml2
  glog
  gflags
  yaml-cpp
)
add_dependencies(export_path 
  ${catkin_EXPORTED_TARGETS}
)

install(TARGETS
  fixed_path 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
  PATTERN ".git" EXCLUDE
)
