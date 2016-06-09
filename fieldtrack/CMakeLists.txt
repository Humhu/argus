cmake_minimum_required(VERSION 2.8.3)
project(fieldtrack)

set(CMAKE_CXX_FLAGS "-g -Wall -std=c++11")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake/")

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED 
	COMPONENTS		message_generation
					roscpp
					std_msgs
					sensor_msgs
					nav_msgs
					argus_utils
					extrinsics_array
					fiducials
					lookup
)

# Use Boost for most utilities, threading
find_package(Boost REQUIRED)

# Use Eigen for matrices, linear algebra
find_package(Eigen3 REQUIRED)

catkin_package(
	INCLUDE_DIRS 	include
	
	LIBRARIES 		fieldtrack
					
	CATKIN_DEPENDS 	message_runtime 
					roscpp 
					std_msgs
					nav_msgs
					argus_utils
					extrinsics_array
					fiducials
					lookup
					
	DEPENDS 		Boost 
					system_lib
)

include_directories( include
					 ${Boost_INCLUDE_DIRS}
					 ${EIGEN3_INCLUDE_DIR}
					 ${catkin_INCLUDE_DIRS}
)

add_library( fieldtrack
	src/ArrayPoseEstimator.cpp
	src/BodyFrameTracker.cpp
	src/FiducialPoseEstimator.cpp
	src/FieldtrackCommon.cpp
	src/FixedFrameTracker.cpp
	src/SimpleStateEstimator.cpp
	src/TargetInfoManager.cpp
	src/VelocityIntegrator.cpp
)
# add_dependencies( fieldtrack ${fieldtrack_EXPORTED_TARGETS} )
target_link_libraries( fieldtrack
	${Boost_LIBRARIES}
	${catkin_LIBRARIES}
)
 
add_executable( array_pose_estimator
	nodes/array_pose_estimator.cpp )
add_dependencies( array_pose_estimator ${PROJECT_NAME}_gencfg )
target_link_libraries( array_pose_estimator
	fieldtrack
	${catkin_LIBRARIES} )

add_executable( fiducial_pose_estimator
	nodes/fiducial_pose_estimator.cpp )
add_dependencies( fiducial_pose_estimator ${PROJECT_NAME}_gencfg )
target_link_libraries( fiducial_pose_estimator
	fieldtrack
	${catkin_LIBRARIES} )

add_executable( fixed_frame_tracker
	nodes/fixed_frame_tracker.cpp )
add_dependencies( fixed_frame_tracker ${PROJECT_NAME}_gencfg )
target_link_libraries( fixed_frame_tracker
	fieldtrack
	${catkin_LIBRARIES} )

add_executable( body_frame_tracker
	nodes/body_frame_tracker.cpp )
add_dependencies( body_frame_tracker ${PROJECT_NAME}_gencfg )
target_link_libraries( body_frame_tracker
	fieldtrack
	${catkin_LIBRARIES} )

add_executable( velocity_integrator_node
	nodes/velocity_integrator_node.cpp )
add_dependencies( velocity_integrator_node ${PROJECT_NAME}_gencfg )
target_link_libraries( velocity_integrator_node
	fieldtrack
	${catkin_LIBRARIES} )

add_executable( pose_reference_resolver
	nodes/pose_reference_resolver.cpp )
add_dependencies( pose_reference_resolver ${PROJECT_NAME}_gencfg )
target_link_libraries( pose_reference_resolver
	fieldtrack
	${catkin_LIBRARIES} )

add_executable( state_estimator_node
	nodes/state_estimator_node.cpp )
add_dependencies( state_estimator_node ${ROJECT_NAME}_gencfg )
target_link_libraries( state_estimator_node
	fieldtrack
	${catkin_LIBRARIES} )

add_executable( odometry_converter_node
	nodes/odometry_converter_node.cpp )
add_dependencies( odometry_converter_node ${ROJECT_NAME}_gencfg )
target_link_libraries( odometry_converter_node
	fieldtrack
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
