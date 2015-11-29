cmake_minimum_required(VERSION 2.8.3)
project(extrinsics_array)

set(CMAKE_CXX_FLAGS "-g -Wall -std=c++11")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake/")

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED 
	COMPONENTS		message_generation
					roscpp
					std_msgs
					geometry_msgs
					argus_utils
					lookup
)

# Use Boost for most utilities, threading
find_package(Boost REQUIRED)

# Find OpenCV3 and set flags for other ROS packages
find_package(OpenCV 3 REQUIRED)
add_definitions( "-DOPENCV3=1" )

# Use Eigen for matrices, linear algebra
find_package(Eigen3 REQUIRED)

add_message_files(
	FILES			ExtrinsicsArrayInfo.msg
)

generate_messages(
	DEPENDENCIES	std_msgs
					geometry_msgs
)

catkin_package(
	INCLUDE_DIRS 	include
	
	LIBRARIES 		lib_extrinsics_array
					
	CATKIN_DEPENDS 	message_runtime 
					roscpp 
					std_msgs
					geometry_msgs
					argus_utils
					lookup
					
	DEPENDS 		Boost 
					system_lib
)

include_directories( include
					 ${Boost_INCLUDE_DIRS}
					 ${EIGEN3_INCLUDE_DIR}
					 ${OpenCV_INCLUDE_DIRS}
					 ${catkin_INCLUDE_DIRS}
)

add_library( lib_extrinsics_array
	src/ExtrinsicsArray.cpp
	src/ExtrinsicsArrayCalibrationParsers.cpp
	src/ExtrinsicsInfoManager.cpp
)
add_dependencies( lib_extrinsics_array ${${PROJECT_NAME}_EXPORTED_TARGETS} )
target_link_libraries( lib_extrinsics_array
	${Boost_LIBRARIES}
	${catkin_LIBRARIES}
)

## Mark executables and/or libraries for installation
install(TARGETS lib_extrinsics_array
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	FILES_MATCHING PATTERN "*.h"
)
