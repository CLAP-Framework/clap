cmake_minimum_required(VERSION 2.8.3)
project(zzz_perception_localization_imu_wheel_odom)

find_package(catkin REQUIRED)
catkin_package()

catkin_python_setup()

catkin_install_python(PROGRAMS
  nodes/locator
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
