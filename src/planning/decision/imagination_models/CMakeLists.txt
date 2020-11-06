cmake_minimum_required(VERSION 2.8.3)
project(zzz_planning_decision_imagination_models)

find_package(catkin REQUIRED)
catkin_package()

catkin_python_setup()

catkin_install_python(PROGRAMS
  nodes/imagination_utility
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
