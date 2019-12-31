cmake_minimum_required(VERSION 2.8.3)
project(zzz_tools_calibration)

find_package(catkin REQUIRED)
catkin_package()

# catkin_python_setup()

catkin_install_python(PROGRAMS
  nodes/broadcaster
  nodes/redirect_intrinsics
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
