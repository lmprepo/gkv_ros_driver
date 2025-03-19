# CMakeList.txt: проект CMake для Receive_ADC_Data_Example; включите исходный код и определения,
# укажите здесь логику для конкретного проекта.
#
cmake_minimum_required (VERSION 3.8)

add_library(GKV_ROS_WrapperLibrary
	    GKV_DeviceROSWrapper.h
	    GKV_DeviceROSWrapper.cpp)

target_link_libraries(GKV_ROS_WrapperLibrary 
			PUBLIC ${catkin_LIBRARIES} 
			PRIVATE LMP_CommunicationLibrary)

target_include_directories(GKV_ROS_WrapperLibrary PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")


