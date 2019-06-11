cmake_minimum_required(VERSION 2.8.3)
project(zzz_perception_msgs)

find_package(catkin REQUIRED genmsg std_msgs # essential dependency
    geometry_msgs sensor_msgs
)

# add_message_files(DIRECTORY msg FILES
#     Placeholder.msg
# )

generate_messages(DEPENDENCIES vision_msgs geometry_msgs sensor_msgs std_msgs)
catkin_package(CATKIN_DEPENDS vision_msgs geometry_msgs sensor_msgs std_msgs)
