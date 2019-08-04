cmake_minimum_required(VERSION 2.8.3)
project(zzz_control_latlon_controllers)

find_package(catkin REQUIRED)
catkin_package()

catkin_python_setup()

catkin_install_python(PROGRAMS
  nodes/pure_persuit
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
