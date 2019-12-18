cmake_minimum_required(VERSION 2.8.3)
project(mcity_msg)

find_package(catkin REQUIRED genmsg std_msgs # essential dependency
    geometry_msgs
)

add_message_files(DIRECTORY msg FILES
    BSM.msg
    BSMVec.msg
    Control.msg
    SPaT.msg
    SPaTVec.msg
    VehicleState.msg
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)
catkin_package(CATKIN_DEPENDS std_msgs geometry_msgs)
