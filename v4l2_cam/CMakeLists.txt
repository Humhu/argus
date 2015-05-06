cmake_minimum_required(VERSION 2.8.3)
project(v4l2_cam)

set(CMAKE_CXX_FLAGS "-g -Wall -std=c++11")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake/")

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED 
	COMPONENTS		camera_info_manager
					cv_bridge
					image_transport
					message_generation
					nodelet
					roscpp
					sensor_msgs
					std_msgs
					resource_management
)

# Use Boost for most utilities, threading
find_package(Boost REQUIRED)

# Find OpenCV3 and set flag
find_package(OpenCV 3 REQUIRED)
add_definitions( "-DOPENCV3=1" )

# Use userspace libv4l2 for interfacing
find_package(V4L2 REQUIRED)

## Generate service files
add_service_files( 
	FILES 			CycleCameras.srv
					PrintCapabilities.srv
					SetStreaming.srv
)

generate_messages(
	DEPENDENCIES	std_msgs
					sensor_msgs
)

catkin_package(
	INCLUDE_DIRS 	include
	
	LIBRARIES 		v4l2_cam
					v4l2_cam_nodelet
					
	CATKIN_DEPENDS 	camera_info_manager
					cv_bridge 
					image_transport 
					message_runtime 
					nodelet
					roscpp 
					sensor_msgs 
					std_msgs 
					resource_management
					
	DEPENDS 		Boost 
					OpenCV 
					system_lib
)

include_directories( include
					 ${Boost_INCLUDE_DIRS}
					 ${OpenCV_INCLUDE_DIRS}
					 ${V4L2_INCLUDE_DIRS}
					 ${catkin_INCLUDE_DIRS}
)

add_library( v4l2_cam
	src/CameraDriver.cpp
	src/CameraManager.cpp
	src/CameraTypes.cpp
	src/DriverNode.cpp
)
add_dependencies( v4l2_cam ${v4l2_cam_EXPORTED_TARGETS} )
target_link_libraries( v4l2_cam
	${Boost_LIBRARIES}
	${OpenCV_LIBS}
	${V4L2_LIBRARIES}
	${catkin_LIBRARIES}
)

add_library( v4l2_cam_nodelet nodes/camera_nodelet.cpp )
add_dependencies( v4l2_cam_nodelet ${v4l2_cam_EXPORTED_TARGETS}
								   "${PROJECT_NAME}_gencfg" 
)
target_link_libraries( v4l2_cam_nodelet v4l2_cam
										${catkin_LIBRARIES} 
)

add_executable( camera_node nodes/camera_node.cpp )
add_dependencies( camera_node ${PROJECT_NAME}_gencfg )
target_link_libraries( camera_node
	${catkin_LIBRARIES}
	v4l2_cam )

add_executable( viewer_node nodes/viewer_node.cpp )
add_dependencies( viewer_node ${PROJECT_NAME}_gencfg )
target_link_libraries( viewer_node
	${catkin_LIBRARIES} )
	
add_executable( camera_manager nodes/camera_manager.cpp )
add_dependencies( camera_manager ${PROJECT_NAME}_gencfg )
target_link_libraries( camera_manager
	${catkin_LIBRARIES}
	v4l2_cam)
	
## Mark executables and/or libraries for installation
install(TARGETS v4l2_cam camera_node viewer_node
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	FILES_MATCHING PATTERN "*.h"
)
