cmake_minimum_required(VERSION 2.8.3)
project(resource_management)

set(CMAKE_CXX_FLAGS "-g -Wall -std=c++11")

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED 
	COMPONENTS		message_generation
					nodelet
					roscpp
					std_msgs
)

# Use Boost for most utilities, threading
find_package(Boost REQUIRED)

## Generate service files
add_message_files(
	FILES			ResourceRequest.msg 
					ResourceGrant.msg 
)

add_service_files( 
	FILES 			ReleaseResources.srv
					RequestResources.srv
)

generate_messages(
	DEPENDENCIES	std_msgs
)

catkin_package(
	INCLUDE_DIRS 	include
	
	LIBRARIES 		resource_management
					
	CATKIN_DEPENDS 	message_runtime 
					nodelet
					roscpp 
					std_msgs 
					
	DEPENDS 		Boost 
					system_lib
)

include_directories( include
					 ${Boost_INCLUDE_DIRS}
					 ${catkin_INCLUDE_DIRS}
)

add_library( resource_management
	src/ResourceManager.cpp
	src/ResourceUser.cpp
)
add_dependencies( resource_management ${resource_management_EXPORTED_TARGETS} )
target_link_libraries( resource_management
					   ${Boost_LIBRARIES}
					   ${catkin_LIBRARIES}
)

add_executable( resource_manager nodes/resource_manager.cpp )
add_dependencies( resource_manager ${PROJECT_NAME}_gencfg )
target_link_libraries( resource_manager
	${catkin_LIBRARIES}
	resource_management )
	
## Mark executables and/or libraries for installation
install(TARGETS resource_management resource_manager
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	FILES_MATCHING PATTERN "*.h"
)
