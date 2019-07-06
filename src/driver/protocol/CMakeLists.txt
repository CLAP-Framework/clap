cmake_minimum_required(VERSION 2.8.3)
project(zzz_driver_msgs)

find_package(catkin REQUIRED genmsg std_msgs actionlib_msgs # essential dependency  
    geometry_msgs nav_msgs
)

add_action_Files(DIRECTORY action FILES
    LoadMap.action
)

add_message_files(DIRECTORY msg FILES
    AuxiliaryReport.msg
    ChassisReport.msg
    ControlReport.msg
    LaneState.msg
    MapState.msg
    ObstacleData.msg
    ObstacleDataArray.msg
    ResourceReport.msg
)

generate_messages(DEPENDENCIES std_msgs actionlib_msgs geometry_msgs nav_msgs)
catkin_package(CATKIN_DEPENDS std_msgs actionlib_msgs geometry_msgs nav_msgs)
