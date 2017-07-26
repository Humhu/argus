cmake_minimum_required(VERSION 2.8.3)
project(dispair)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED 
	COMPONENTS		roscpp
					sensor_msgs
					stereo_msgs					
					geometry_msgs
					nav_msgs					
					cv_bridge
					image_transport
					image_geometry
					argus_utils
					camplex
					message_filters
					dynamic_reconfigure
)

message( STATUS ${CMAKE_SOURCE_DIR} )
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake/")

find_package(Boost REQUIRED)

include( ExternalProject )
ExternalProject_Add( bpvo
                     GIT_REPOSITORY   https://github.com/Humhu/bpvo.git
                     BUILD_COMMAND    cmake --build .
                     INSTALL_COMMAND  make install
                     CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CATKIN_DEVEL_PREFIX}
)
file( MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include/bpvo )
set( bpvo_INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include/bpvo )
set( bpvo_LIBRARIES ${CATKIN_DEVEL_PREFIX}/lib/libbpvo.so ${CATKIN_DEVEL_PREFIX}/lib/libbpvo_utils.so )

# Find OpenCV3 and set flags for other ROS packages
find_package(OpenCV 3 REQUIRED)
add_definitions( "-DOPENCV3=1" )

generate_dynamic_reconfigure_options(
  cfg/vo_disparity.cfg
)

catkin_package(
	INCLUDE_DIRS 	include
					${bpvo_INCLUDE_DIRS}

	LIBRARIES 		dispair
					${bpvo_LIBRARIES}

	CATKIN_DEPENDS 	roscpp 
					sensor_msgs 
					stereo_msgs
					geometry_msgs
					nav_msgs
					cv_bridge 
					image_transport 
					message_runtime
					argus_utils
					camplex
					message_filters
					dynamic_reconfigure								
					
	DEPENDS 		Boost 
					OpenCV 
)

include_directories( include
	${bpvo_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "-g -Wall -std=c++0x")

add_library( dispair
	src/ImageSync.cpp
)
target_link_libraries( dispair
	${OpenCV_LIBS}
	${catkin_LIBRARIES}
	${bpvo_LIBRARIES}
)
add_dependencies( dispair bpvo dispair_gencfg )

add_executable( disparity_vo_node nodes/disparity_vo.cpp )
target_link_libraries( disparity_vo_node dispair ${catkin_LIBRARIES} ${OpenCV_LIBS} ${bpvo_LIBRARIES} )
add_dependencies( disparity_vo_node dispair_gencfg)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS dispair 
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	FILES_MATCHING PATTERN "*.h"
)
