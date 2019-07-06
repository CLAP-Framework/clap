cmake_minimum_required(VERSION 2.8.3)
project(zzz_cognition_dynamic_local_map)

find_package(catkin REQUIRED)
catkin_package()

catkin_python_setup()

catkin_install_python(PROGRAMS nodes/dynamic_map_builder
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
