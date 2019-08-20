cmake_minimum_required(VERSION 2.8.3)
project(zzz_driver_simulators_carla_challenge)

find_package(catkin REQUIRED)
catkin_package()

catkin_install_python(PROGRAMS
  nodes/generate
  scripts/spawn_agent
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
