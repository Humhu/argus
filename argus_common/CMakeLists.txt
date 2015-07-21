cmake_minimum_required(VERSION 2.8.3)
project(argus_common)

set(CMAKE_CXX_FLAGS "-g -Wall -std=c++11")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake/")

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED 
	COMPONENTS		message_generation
					roscpp
					std_msgs
)

# Use Boost for most utilities, threading
find_package(Boost REQUIRED)

find_package(Eigen3 REQUIRED)


## Generate service files

catkin_package(
	INCLUDE_DIRS 	include
	
	LIBRARIES 		argus_common
					
	CATKIN_DEPENDS 	message_runtime 
					roscpp 
					std_msgs 
					
	DEPENDS 		Boost 
					system_lib
)

include_directories( include
					 ${Boost_INCLUDE_DIRS}
					 ${EIGEN3_INCLUDE_DIR}
					 ${catkin_INCLUDE_DIRS}
)

add_library( argus_common
	src/PoseSE2.cpp
	src/PoseSE3.cpp
	src/YamlUtils.cpp
)
# add_dependencies( argus_common ${argus_common_EXPORTED_TARGETS} )
target_link_libraries( argus_common
	${Boost_LIBRARIES}
	${catkin_LIBRARIES}
	yaml-cpp
)


## Mark executables and/or libraries for installation
install(TARGETS argus_common 
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

file(GLOB argus_common_HEADERS
	"include/${PROJECT_NAME}/*.h"
	"include/${PROJECT_NAME}/*.hpp"
)
install(FILES ${argus_common_HEADERS}
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
