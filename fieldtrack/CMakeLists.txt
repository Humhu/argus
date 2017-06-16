cmake_minimum_required(VERSION 2.8.3)
project(fieldtrack)

set(CMAKE_CXX_FLAGS "-g -Wall -std=c++0x")

if (NOT DEFINED CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "")
  message(STATUS "No build type selected, default to Release")
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
endif ()
message(STATUS "Build configuration: ${CMAKE_BUILD_TYPE}")

# Finding dependencies
# ====================
find_package( catkin REQUIRED 
              COMPONENTS message_generation
                         roscpp
                         std_msgs 
                         nav_msgs
                         argus_utils
                         extrinsics_array
                         lookup
						 modprop
)

# Use Boost for most utilities, threading
find_package( Boost REQUIRED )

# Messages and services
# =====================
add_service_files( FILES ResetFilter.srv 
)

add_message_files( FILES TargetState.msg 
)

generate_messages( DEPENDENCIES std_msgs
                                geometry_msgs )

# Package configuration
# =====================
catkin_package( INCLUDE_DIRS   include

                LIBRARIES      fieldtrack

                CATKIN_DEPENDS message_runtime 
                               roscpp
                               std_msgs
                               nav_msgs
                               argus_utils
                               extrinsics_array
                               lookup
							   modprop

                DEPENDS        Boost
                               system_lib
)

include_directories( include
                     ${Boost_INCLUDE_DIRS}
                     ${catkin_INCLUDE_DIRS}
)

add_library( fieldtrack
    src/AdaptiveCovarianceEstimator.cpp
	src/BufferedEstimator.cpp
	src/CovarianceModels.cpp
	src/DimensionParser.cpp
	src/FieldtrackCommon.cpp
	src/KalmanChain.cpp
	src/LikelihoodChain.cpp
	src/NoiseLearner.cpp
	# src/ObservationSourceManager.cpp
	# src/PoseDerivativeFilter.cpp
	src/PoseEstimator.cpp
	src/PoseSourceManager.cpp
	# src/StateEstimator.cpp
	src/TargetInfoManager.cpp
	src/VelocityEstimator.cpp
	src/VelocitySourceManager.cpp
)
target_link_libraries( fieldtrack
	${Boost_LIBRARIES}
	${catkin_LIBRARIES}
)
add_dependencies( fieldtrack ${catkin_EXPORTED_TARGETS} )

# add_executable( state_estimator_node
# 	nodes/state_estimator_node.cpp )
# target_link_libraries( state_estimator_node
# 	fieldtrack
# 	${catkin_LIBRARIES} )

add_executable( velocity_estimator_node
	nodes/velocity_estimator_node.cpp )
target_link_libraries( velocity_estimator_node
	fieldtrack
	${catkin_LIBRARIES} )

add_executable( pose_estimator_node
	nodes/pose_estimator_node.cpp )
target_link_libraries( pose_estimator_node
	fieldtrack
	${catkin_LIBRARIES} )

add_executable( odometry_converter_node
	nodes/odometry_converter_node.cpp )
target_link_libraries( odometry_converter_node
	fieldtrack
	${catkin_LIBRARIES} )

add_executable( velocity_converter_node
  nodes/velocity_converter_node.cpp )
target_link_libraries( velocity_converter_node
  fieldtrack
  ${catkin_LIBRARIES} )

add_executable( odometry_differentiator_node
	nodes/odometry_differentiator.cpp )
target_link_libraries( odometry_differentiator_node
	fieldtrack
	${catkin_LIBRARIES} )

add_executable( test_buff_cov_estimator tests/test_buffer_cov_estimator.cpp )
target_link_libraries( test_buff_cov_estimator
	fieldtrack 
	${catkin_LIBRARIES} )

## Mark executables and/or libraries for installation
install(TARGETS velocity_estimator_node
                odometry_converter_node
                odometry_differentiator_node
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	FILES_MATCHING PATTERN "*.h"
)
