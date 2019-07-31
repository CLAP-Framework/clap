cmake_minimum_required(VERSION 2.8.3)
project(zzz_control_pure_persuit_control)

find_package(catkin REQUIRED)
catkin_package()

catkin_python_setup()

catkin_install_python(PROGRAMS nodes/pure_persuit_controller_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
