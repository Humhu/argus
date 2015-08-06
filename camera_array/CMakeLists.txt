cmake_minimum_required(VERSION 2.8.3)
project(camera_array)

set(CMAKE_CXX_FLAGS "-g -Wall -std=c++11")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake/")

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED 
	COMPONENTS		message_generation
					nodelet
					roscpp
					std_msgs
					argus_utils
					camplex
)

# Use Boost for most utilities, threading
find_package(Boost REQUIRED)

add_service_files( 
	FILES 			CycleArray.srv
					DisableArray.srv
					DisableArrayCamera.srv
					EnableArrayCamera.srv
					ListArrayCameras.srv
)

generate_messages(
	DEPENDENCIES	std_msgs
)

catkin_package(
	INCLUDE_DIRS 	include
	
	LIBRARIES 		camera_array
					
	CATKIN_DEPENDS 	message_runtime 
					nodelet
					roscpp 
					std_msgs
					argus_utils
					camplex
					
	DEPENDS 		Boost 
					system_lib
)

include_directories( include
					 ${Boost_INCLUDE_DIRS}
					 ${catkin_INCLUDE_DIRS}
)

add_library( camera_array
	src/CameraArray.cpp
)
add_dependencies( camera_array ${camera_array_EXPORTED_TARGETS} )
target_link_libraries( camera_array
	${Boost_LIBRARIES}
	${catkin_LIBRARIES}
)

add_library( camera_array_nodelet 
	nodes/camera_array_nodelet.cpp )
add_dependencies( camera_array_nodelet 
	${camera_array_EXPORTED_TARGETS}
	  "${PROJECT_NAME}_gencfg" 
)
target_link_libraries( camera_array_nodelet 
	camera_array
	${catkin_LIBRARIES} 
)

add_executable( camera_array_node
	nodes/camera_array_node.cpp )
add_dependencies( camera_array_node 
	${PROJECT_NAME}_gencfg )
target_link_libraries( camera_array_node
	${catkin_LIBRARIES}
	camera_array )

## Mark executables and/or libraries for installation
install(TARGETS 
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	FILES_MATCHING PATTERN "*.h"
)
