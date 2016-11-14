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
					argus_utils
					argus_msgs
					fiducials
					camplex
)

message( STATUS ${CATKIN_DEVEL_PREFIX} )

include( ExternalProject )
ExternalProject_Add(
	apriltags_external
	GIT_REPOSITORY https://github.com/Humhu/apriltags.git #git@github.com:Humhu/apriltags.git
	BUILD_COMMAND cmake --build .
	INSTALL_COMMAND make install
	CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CATKIN_DEVEL_PREFIX}
)
set( apriltags_INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include )
set( apriltags_LIBRARIES ${CATKIN_DEVEL_PREFIX}/lib/libapriltags.so )

find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
	INCLUDE_DIRS 	include
					# TODO It's not happy about this include - change the install path?
					${apriltags_INCLUDE_DIRS}
	
	LIBRARIES 		atags
					${apriltags_LIBRARIES}
					
	CATKIN_DEPENDS 	camera_info_manager
					cv_bridge 
					image_transport 
					message_runtime 
					nodelet
					roscpp 
					sensor_msgs 
					std_msgs
					argus_utils
					argus_msgs
					fiducials
					camplex
					
	DEPENDS 		Boost 
					system_lib
					apriltags_external
)

include_directories( include
					 ${apriltags_INCLUDE_DIRS}
					 ${Boost_INCLUDE_DIRS}
					 ${EIGEN3_INCLUDE_DIR}
					 ${catkin_INCLUDE_DIRS}
)

add_library( atags
	src/AtagCommon.cpp
)
target_link_libraries( atags
	${apriltags_LIBRARIES}
	${Boost_LIBRARIES}
	${catkin_LIBRARIES}
)
add_dependencies( atags apriltags_external ${catkin_EXPORTED_TARGETS} )

# add_library( detector_nodelet nodes/atag_detector_nodelet.cpp )
# add_dependencies( detector_nodelet tags_gencfg 
# 								   apriltags_external 
# )
# target_link_libraries( detector_nodelet atags
# 										${catkin_LIBRARIES} 
# )
	
add_executable( detector_node nodes/AtagDetectorNode.cpp )
add_dependencies( detector_node atags_gencfg
								apriltags_external 
)
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

install(DIRECTORY include/apriltags/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	FILES_MATCHING PATTERN "*.h"
)
