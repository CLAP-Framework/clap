cmake_minimum_required(VERSION 2.8.3)
project(zzz_driver_platforms_mcity_adapter)

find_package(catkin REQUIRED genmsg std_msgs # essential dependency
    geometry_msgs
)
catkin_package()

add_message_files(DIRECTORY msg FILES
    BSM.msg
    BSMVec.msg
    Control.msg
    SPaT.msg
    SPaTVec.msg
    VehicleState.msg
)

catkin_install_python(PROGRAMS
  nodes/convert_topic
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
