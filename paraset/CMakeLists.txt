cmake_minimum_required(VERSION 2.8.3)
project(paraset)

set(CMAKE_CXX_FLAGS "-g -Wall -std=c++11")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake/")

set(CMAKE_BUILD_TYPE Release)

find_package(percepto REQUIRED)
message( STATUS "percepto found at: " ${percepto_INCLUDE_DIRS} )
include_directories( ${percepto_INCLUDE_DIRS} )
list( APPEND paraset_LIBS ${percepto_LIBS} )

find_package(catkin REQUIRED 
	COMPONENTS		message_generation
					roscpp
					argus_utils
					broadcast
					std_msgs
					nav_msgs
)

# Use Boost for most utilities, threading
find_package(Boost REQUIRED COMPONENTS random)

# TODO Figure out how to make this included with argus_utils includes
# Use Eigen for matrices, linear algebra
find_package(Eigen3 REQUIRED)

add_message_files( FILES
    RewardStamped.msg
	RuntimeParameter.msg
)

# Services
add_service_files( FILES
	SetRuntimeParameter.srv
	GetParameterInfo.srv
)

generate_messages( 
	DEPENDENCIES 	std_msgs )

catkin_package(
	INCLUDE_DIRS 	include
						
	LIBRARIES		paraset

	CATKIN_DEPENDS 	roscpp 
					argus_utils
					message_runtime
					broadcast
					std_msgs
					nav_msgs
					
	DEPENDS 		Boost 
					system_lib
)

add_library( paraset
            src/ContinuousPolicy.cpp
            src/ContinuousPolicyManager.cpp
            src/DiscretePolicy.cpp
            src/DiscretePolicyManager.cpp
			src/ParasetCommon.cpp
			src/ParamChecks.cpp
			src/PolicyModules.cpp )
add_dependencies( paraset paraset_gencfg ${paraset_EXPORTED_TARGETS} )
target_link_libraries( paraset
                       ${catkin_LIBRARIES} )

include_directories( include
${EIGEN3_INCLUDE_DIR}
					 ${Boost_INCLUDE_DIRS}
					 ${catkin_INCLUDE_DIRS}
)

add_executable( discrete_policy_node nodes/discrete_policy_node.cpp )
add_dependencies( discrete_policy_node ${PROJECT_NAME}_gencfg )
target_link_libraries( discrete_policy_node
                      ${catkin_LIBRARIES}
                      ${Boost_LIBRARIES}
                      paraset )

add_executable( continuous_policy_node nodes/continuous_policy_node.cpp )
add_dependencies( continuous_policy_node ${PROJECT_NAME}_gencfg )
target_link_libraries( continuous_policy_node
                      ${catkin_LIBRARIES}
                      ${Boost_LIBRARIES}
                      paraset )

install( TARGETS paraset discrete_policy_node continuous_policy_node
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables and/or libraries for installation
install(DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	FILES_MATCHING PATTERN "*.h *.hpp"
)
