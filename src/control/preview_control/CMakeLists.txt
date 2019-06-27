cmake_minimum_required(VERSION 2.8.3)
project(zzz_control_preview_control)

find_package(catkin REQUIRED)
catkin_package()

catkin_python_setup()

catkin_install_python(PROGRAMS nodes/preview_controller
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
