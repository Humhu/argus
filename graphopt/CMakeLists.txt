cmake_minimum_required(VERSION 2.8.3)
project(graphopt)

set(CMAKE_CXX_FLAGS "-g -Wall -std=c++0x")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake/")

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED 
	COMPONENTS		roscpp
					std_msgs
					argus_utils
					argus_msgs
)

message( STATUS ${CATKIN_DEVEL_PREFIX} )

include( ExternalProject )
ExternalProject_Add(
	isam_external
	GIT_REPOSITORY https://github.com/Humhu/isam.git
	BUILD_COMMAND cmake --build .
	INSTALL_COMMAND make install
	CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CATKIN_DEVEL_PREFIX}
)
set( isam_INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include )
set( isam_LIBRARIES ${CATKIN_DEVEL_PREFIX}/lib/libisam.so )

find_package(Boost REQUIRED)

catkin_package(
	INCLUDE_DIRS    include
	
	LIBRARIES       graphopt
	                ${isam_LIBRARIES}

	CATKIN_DEPENDS 	roscpp 
	                std_msgs
	                argus_utils

	DEPENDS         Boost 
	                system_lib
	                isam_external
)

include_directories( include
                     ${Boost_INCLUDE_DIRS}
                     ${catkin_INCLUDE_DIRS}
)

add_library( graphopt
             src/GraphOptimizer.cpp
)
add_dependencies( graphopt isam_external 
                           ${catkin_EXPORTED_TARGETS} )
target_link_libraries( graphopt
                       ${isam_LIBRARIES}
                       ${Boost_LIBRARIES}
                       ${catkin_LIBRARIES}
)

# Mark executables and/or libraries for installation
install(TARGETS graphopt
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY include/isam/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	FILES_MATCHING PATTERN "*.h"
)
