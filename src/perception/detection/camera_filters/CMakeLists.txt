cmake_minimum_required(VERSION 2.8.3)
project(zzz_perception_detection_camera_filters)

find_package(catkin REQUIRED)
catkin_package()

catkin_python_setup()

catkin_install_python(PROGRAMS
  nodes/ipm_projector
  nodes/pseudo_lidar_generator
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
