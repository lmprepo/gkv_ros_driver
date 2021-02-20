cmake_minimum_required (VERSION 3.8)

add_executable(gkv_controller gkv_controller.cpp)
target_link_libraries(gkv_controller 
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
