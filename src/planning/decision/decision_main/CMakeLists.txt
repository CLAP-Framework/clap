cmake_minimum_required(VERSION 2.8.3)
project(zzz_planning_decision_decision_main)

find_package(catkin REQUIRED)
catkin_package()

catkin_python_setup()

catkin_install_python(PROGRAMS nodes/decision_main_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
