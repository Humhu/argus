cmake_minimum_required(VERSION 2.8.3)
project(atags)

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
					image_geometry
					argus_common
					argus_msgs
)

# Use Boost for most utilities, threading
find_package(Boost REQUIRED)

# Find externally built apriltags library
# TODO Use /internal copy if not found?
find_package(apriltags REQUIRED)
include_directories( ${apriltags_INCLUDE_DIRS} )
add_definitions( ${apriltags_DEFINITIONS} )

catkin_package(
	INCLUDE_DIRS 	include
	
	LIBRARIES 		atags
					
	CATKIN_DEPENDS 	camera_info_manager
					cv_bridge 
					image_transport 
					message_runtime 
					nodelet
					roscpp 
					sensor_msgs 
					std_msgs
					image_geometry
					argus_common
					
	DEPENDS 		Boost 
					apriltags
					system_lib
)

include_directories( include
					 ${Boost_INCLUDE_DIRS}
					 ${catkin_INCLUDE_DIRS}
)

add_library( atags
	src/AtagDetector.cpp
	src/AtagCommon.cpp
)
target_link_libraries( atags
	${apriltags_LIBRARIES}
	${Boost_LIBRARIES}
	${catkin_LIBRARIES}
)

add_library( detector_nodelet nodes/atag_detector_nodelet.cpp )
add_dependencies( detector_nodelet "${PROJECT_NAME}_gencfg" )
target_link_libraries( detector_nodelet atags
										 ${catkin_LIBRARIES} 
)
	
add_executable( detector_node nodes/atag_detector_node.cpp )
add_dependencies( detector_node "${PROJECT_NAME}_gencfg" )
target_link_libraries( detector_node
	atags
	${catkin_LIBRARIES}
)
	
## Mark executables and/or libraries for installation
install(TARGETS atags
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	FILES_MATCHING PATTERN "*.h"
)
