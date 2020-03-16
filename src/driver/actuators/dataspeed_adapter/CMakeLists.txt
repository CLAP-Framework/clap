cmake_minimum_required(VERSION 2.8.3)
project(zzz_driver_actuators_dataspeed_adapter)

find_package(catkin REQUIRED)
catkin_package()

catkin_python_setup()

catkin_install_python(PROGRAMS
  nodes/convert_mkz_command
  nodes/convert_mkz_report
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
