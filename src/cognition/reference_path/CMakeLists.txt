cmake_minimum_required(VERSION 2.8.3)
project(zzz_cognition_reference_path)

find_package(catkin REQUIRED)
catkin_package()

catkin_python_setup()

catkin_install_python(PROGRAMS nodes/put_buffer
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
