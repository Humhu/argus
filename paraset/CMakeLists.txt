cmake_minimum_required(VERSION 2.8.3)
project(paraset)

set(CMAKE_CXX_FLAGS "-g -Wall -std=c++11")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake/")

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED 
	COMPONENTS		message_generation
					roscpp
					argus_utils
)

# Use Boost for most utilities, threading
find_package(Boost REQUIRED)

# TODO Figure out how to make this included with argus_utils includes
# Use Eigen for matrices, linear algebra
find_package(Eigen3 REQUIRED)

# Services
add_service_files( FILES
	SetRuntimeParameter.srv
	GetParameterInfo.srv
)

generate_messages()

catkin_package(
	INCLUDE_DIRS 	include
						
	CATKIN_DEPENDS 	roscpp 
					argus_utils
					message_generation
					
	DEPENDS 		Boost 
					system_lib
)

include_directories( include
${EIGEN3_INCLUDE_DIR}
					 ${Boost_INCLUDE_DIRS}
					 ${catkin_INCLUDE_DIRS}
)

## Mark executables and/or libraries for installation
install(DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	FILES_MATCHING PATTERN "*.h *.hpp"
)
