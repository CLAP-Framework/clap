add_compile_options(-O2)

add_library(rslidar_input_first input.cc)
target_link_libraries(rslidar_input_first
  ${catkin_LIBRARIES}
  ${libpcap_LIBRARIES})

  add_library(sem_object_point_cloud_first sem_object.cpp)
  target_link_libraries(sem_object_point_cloud_first
      ${catkin_LIBRARIES})  

add_library(shm_object_point_cloud_first shm_object.cpp)
  target_link_libraries(shm_object_point_cloud_first
      ${catkin_LIBRARIES})

add_library(rslidar_driver_first rsdriver.cpp)
target_link_libraries(rslidar_driver_first
  rslidar_input_first
  shm_object_point_cloud_first
  sem_object_point_cloud_first
  ${catkin_LIBRARIES})

# build the nodelet version
# add_library(driver_nodelet nodelet.cc rsdriver.cpp)
# target_link_libraries(driver_nodelet
#   rslidar_input_first
#   ${catkin_LIBRARIES}
# )

add_executable(rslidar_node_first rslidar_node.cpp)

if(catkin_EXPORTED_TARGETS)
  add_dependencies(rslidar_input_first ${catkin_EXPORTED_TARGETS})
endif()

target_link_libraries(rslidar_node_first
	rslidar_driver_first
  rslidar_input_first
	${catkin_LIBRARIES}
	${libpcap_LIBRARIES}
)

