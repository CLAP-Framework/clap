cmake_minimum_required(VERSION 2.8.3)
project(zzz_driver_simulators_carla_adapter)

find_package(catkin REQUIRED)
catkin_package()

catkin_install_python(PROGRAMS
  nodes/convert_topic
  scripts/spawn_agent
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
