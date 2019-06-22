cmake_minimum_required(VERSION 2.8.3)
project(zzz_library_dynamic_models)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
