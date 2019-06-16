cmake_minimum_required(VERSION 2.8.3)
project(zzz_driver_msgs)

find_package(catkin REQUIRED genmsg std_msgs # essential dependency  
)

add_message_files(DIRECTORY msg FILES
    AuxiliaryReport.msg
    ChassisReport.msg
    ControlReport.msg
    ResourceReport.msg
)

generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS std_msgs)
