cmake_minimum_required(VERSION 2.8.3)
project(camera_array)

set(CMAKE_CXX_FLAGS "-g -Wall -std=c++11")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake/")

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED 
	COMPONENTS		roscpp
					message_generation
					std_msgs
					nav_msgs
					visualization_msgs
					argus_utils
					extrinsics_array
					image_geometry
					lookup
					fieldtrack
					camplex
					fiducials
)

add_message_files(
	FILES		CameraStatus.msg
				CameraArrayStatus.msg 
)

generate_messages(
	DEPENDENCIES 	std_msgs
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
					message_runtime
					std_msgs
					nav_msgs
					visualization_msgs
					argus_utils
					extrinsics_array
					image_geometry
					lookup
					fieldtrack
					camplex
					fiducials
	
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
	src/ExpectationPolicy.cpp
	src/FiducialDetectionModel.cpp
	src/FiducialRewardFunction.cpp
	src/PolicyManager.cpp
	src/RandomArrayManager.cpp
	src/SystemDistributions.cpp
	src/SystemModels.cpp
	src/SystemStates.cpp
)
add_dependencies( camera_array ${camera_array_EXPORTED_TARGETS} )
target_link_libraries( camera_array
	${Boost_LIBRARIES}
	${catkin_LIBRARIES}
)

add_executable( camera_array_manager nodes/camera_array_manager.cpp )
add_dependencies( camera_array_manager camera_array_gencfg )
target_link_libraries( camera_array_manager camera_array
											${catkin_LIBRARIES} )

add_executable( array_visualizer nodes/array_visualizer.cpp )
add_dependencies( array_visualizer camera_array_gencfg )
target_link_libraries( array_visualizer camera_array
										${catkin_LIBRARIES} )

add_executable( array_synchronizer_node nodes/array_synchronizer_node.cpp )
add_dependencies( array_synchronizer_node ${PROJECT_NAME}_gencfg )
target_link_libraries( array_synchronizer_node
                       ${isam_LIBRARIES}
                       ${catkin_LIBRARIES}
                       camera_array
)

## Mark executables and/or libraries for installation
install(TARGETS camera_array_manager
                array_visualizer
                array_synchronizer_node
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	FILES_MATCHING PATTERN "*.h"
)
