cmake_minimum_required(VERSION 2.8.3)
project(zzz_driver_simulators_sumo_adapter)

find_package(catkin REQUIRED)
catkin_package()

catkin_python_setup()

catkin_install_python(PROGRAMS nodes/map_provider
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
