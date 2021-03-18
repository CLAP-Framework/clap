cmake_minimum_required(VERSION 3.0.0)
project(obu_trafficlight VERSION 0.1.0)

add_compile_options(-std=c++14)
# add_compile_options(-g)
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -std=c++14")
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  zzz_perception_msgs
)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  std_msgs
  zzz_perception_msgs
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    third_party/jeayeson/include
)

add_executable (${PROJECT_NAME} 
    src/main.cc
)
target_link_libraries(${PROJECT_NAME} 
${catkin_LIBRARIES}
)
add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)


