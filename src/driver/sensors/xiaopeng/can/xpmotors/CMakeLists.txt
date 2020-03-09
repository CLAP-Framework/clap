cmake_minimum_required(VERSION 2.8.3)
project(xpmotors)

add_compile_options(-std=c++11)

find_package(Boost REQUIRED COMPONENTS system filesystem)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp xpmotors_can_msgs std_msgs message_generation  geodesy
std_msgs geometry_msgs nav_msgs)

find_package(Eigen3 REQUIRED)

## pkg-config libraries
# find_package(PkgConfig REQUIRED)



## Generate messages in the 'msg' folder
#add_message_files(DIRECTORY msg FILES
#DecisionTrajectory.msg
#)

## Generate added messages and services with any dependencies listed here
generate_messages(
DEPENDENCIES
std_msgs  # Or other packages containing msgs
geometry_msgs
nav_msgs
)
###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES xpmotors
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

## build can-net udp library 
add_library(${PROJECT_NAME}
   src/XUdp.cpp
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
#add_dependencies(${PROJECT_NAME} xpmotors_can_msgs_gencpp) 

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(auto_control_node nodes/auto_control_node.cpp)
add_executable(auto_state_node nodes/auto_state_node.cpp)
add_executable(speedpub nodes/speedpub.cpp)
add_executable(follower_cz nodes/follower_cz.cpp)
add_executable(getPath2txt nodes/getPath2txt.cpp)
add_executable(GPS2ego_pose nodes/GPS2ego_pose.cpp)

## Rename C++ executable without prefixGPS2ego_pose
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(auto_control_node ${catkin_LIBRARIES} ${PROJECT_NAME})
target_link_libraries(auto_state_node ${catkin_LIBRARIES} ${PROJECT_NAME})
target_link_libraries(speedpub ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PROJECT_NAME})
target_link_libraries(follower_cz ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PROJECT_NAME})
target_link_libraries(getPath2txt ${catkin_LIBRARIES} ${PROJECT_NAME})
target_link_libraries(GPS2ego_pose ${catkin_LIBRARIES}  ${Boost_LIBRARIES} ${PROJECT_NAME})
#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_xpmotors.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
