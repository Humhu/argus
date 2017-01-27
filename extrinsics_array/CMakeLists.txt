cmake_minimum_required(VERSION 2.8.3)
project(extrinsics_array)

set(CMAKE_CXX_FLAGS "-g -Wall -std=c++11")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake/")

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED 
	COMPONENTS		roscpp
					argus_utils
					lookup
					tf2_ros
)

# Use Boost for most utilities, threading
find_package(Boost REQUIRED)

# Use Eigen for matrices, linear algebra
find_package(Eigen3 REQUIRED)

catkin_package(
	INCLUDE_DIRS 	include
	
	LIBRARIES 		extrinsics_array
					
	CATKIN_DEPENDS 	roscpp 
					argus_utils
					lookup
					tf2_ros
					
	DEPENDS 		Boost 
					system_lib
)

include_directories( include
					 ${Boost_INCLUDE_DIRS}
					 ${EIGEN3_INCLUDE_DIR}
					 ${OpenCV_INCLUDE_DIRS}
					 ${catkin_INCLUDE_DIRS}
)

add_library( extrinsics_array
	src/ExtrinsicsCommon.cpp
	src/ExtrinsicsInterface.cpp
)
target_link_libraries( extrinsics_array
	${Boost_LIBRARIES}
	${catkin_LIBRARIES}
)

add_executable( extrinsics_publisher_node nodes/extrinsics_publisher_node.cpp )
add_dependencies( extrinsics_publisher_node extrinsics_array 
                                            ${catkin_EXPORTED_TARGETS} )
target_link_libraries( extrinsics_publisher_node extrinsics_array
                                                 ${catkin_LIBRARIES} )

add_executable( transform_repeater_node nodes/transform_repeater_node.cpp )
add_dependencies( transform_repeater_node extrinsics_array 
                                            ${catkin_EXPORTED_TARGETS} )
target_link_libraries( transform_repeater_node extrinsics_array
                                                 ${catkin_LIBRARIES} )

## Mark executables and/or libraries for installation
install(TARGETS extrinsics_array 
                extrinsics_publisher_node
                transform_repeater_node
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	FILES_MATCHING PATTERN "*.h"
)
