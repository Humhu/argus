cmake_minimum_required(VERSION 2.8.3)
project(camera_array)

set(CMAKE_CXX_FLAGS "-g -Wall -std=c++11")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake/")

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED 
	COMPONENTS		roscpp
					argus_utils
					extrinsics_array
					image_geometry
					lookup
					nav_msgs
)

# Use Boost for most utilities, threading
find_package(Boost REQUIRED 
	COMPONENTS random )

# Use Eigen for matrices, linear algebra
find_package(Eigen3 REQUIRED)

catkin_package(
	INCLUDE_DIRS 	include
	
	LIBRARIES 		camera_array
	
	CATKIN_DEPENDS 	roscpp 
					argus_utils
					extrinsics_array
					image_geometry
					lookup
					nav_msgs
	
	DEPENDS 		Boost 
					system_lib
)

include_directories( include
					 ${Boost_INCLUDE_DIRS}
					 ${EIGEN3_INCLUDE_DIR}
					 ${catkin_INCLUDE_DIRS}
)

add_library( camera_array
	src/CameraArrayManager.cpp
	src/RandomArrayManager.cpp
)
target_link_libraries( camera_array
	${Boost_LIBRARIES}
	${catkin_LIBRARIES}
)

add_executable( camera_array_manager nodes/camera_array_manager.cpp )
add_dependencies( camera_array_manager camera_array_gencfg )
target_link_libraries( camera_array_manager camera_array
											${catkin_LIBRARIES} )

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
