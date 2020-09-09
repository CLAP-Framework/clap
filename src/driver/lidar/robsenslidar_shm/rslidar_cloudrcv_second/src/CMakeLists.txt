add_compile_options(-O2)

add_library(rslidar_input_second input.cc)
target_link_libraries(rslidar_input_second
  ${catkin_LIBRARIES}
  ${libpcap_LIBRARIES})

  add_library(sem_object_point_cloud_second sem_object.cpp)
  target_link_libraries(sem_object_point_cloud_second
      ${catkin_LIBRARIES})  

add_library(shm_object_point_cloud_second shm_object.cpp)
  target_link_libraries(shm_object_point_cloud_second
      ${catkin_LIBRARIES})

add_library(rslidar_driver_second rsdriver.cpp)
target_link_libraries(rslidar_driver_second
  rslidar_input_second
  shm_object_point_cloud_second
  sem_object_point_cloud_second
  ${catkin_LIBRARIES})

# build the nodelet version
# add_library(driver_nodelet_second nodelet.cc rsdriver.cpp)
# target_link_libraries(driver_nodelet_second
#   rslidar_input_second
#   ${catkin_LIBRARIES}
# )

add_executable(rslidar_node_second rslidar_node.cpp)

if(catkin_EXPORTED_TARGETS)
  add_dependencies(rslidar_input_second ${catkin_EXPORTED_TARGETS})
endif()

target_link_libraries(rslidar_node_second
	rslidar_driver_second
  rslidar_input_second
	${catkin_LIBRARIES}
	${libpcap_LIBRARIES}
)

