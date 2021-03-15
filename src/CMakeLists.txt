cmake_minimum_required (VERSION 3.8)

add_executable(gkv_controller gkv_controller.cpp)
target_link_libraries(gkv_controller 
			PUBLIC ${catkin_LIBRARIES} 
			PRIVATE GKV_DevCommunicationLibrary
			PRIVATE GKV_ROS_WrapperLibrary)

add_executable(gkv_controller_ros_mode gkv_controller_ros_mode.cpp)
target_link_libraries(gkv_controller_ros_mode
			PUBLIC ${catkin_LIBRARIES} 
			PRIVATE GKV_DevCommunicationLibrary
			PRIVATE GKV_ROS_WrapperLibrary)

add_executable(gkv_listener gkv_listener.cpp)
target_link_libraries(gkv_listener 
		PUBLIC ${catkin_LIBRARIES}
		PRIVATE GKV_DevCommunicationLibrary)

add_executable(gkv_alg_switcher gkv_alg_switcher.cpp)
target_link_libraries(gkv_alg_switcher 
		PUBLIC ${catkin_LIBRARIES}
		PRIVATE GKV_DevCommunicationLibrary)

add_executable(gkv_custom_packet_listener gkv_custom_packet_listener.cpp)
target_link_libraries(gkv_custom_packet_listener 
		PUBLIC ${catkin_LIBRARIES}
		PRIVATE GKV_DevCommunicationLibrary)
