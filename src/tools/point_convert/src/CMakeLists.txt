add_library(rslidar_convert_data rawdata.cc)
target_link_libraries(rslidar_convert_data
    ${catkin_LIBRARIES}
    ${libpcap_LIBRARIES})

add_library(rslidar_convert_point convert.cc)
target_link_libraries(rslidar_convert_point
rslidar_convert_data
    ${catkin_LIBRARIES})


add_executable(point_convert_node cloud_node.cc)

if(catkin_EXPORTED_TARGETS)
  add_dependencies(rslidar_convert_data ${catkin_EXPORTED_TARGETS})
endif()

target_link_libraries(point_convert_node
    rslidar_convert_point
    ${catkin_LIBRARIES}
    ${libpcap_LIBRARIES})
