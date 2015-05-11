cmake_minimum_required(VERSION 2.8.3)
project(odoflow)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED 
	COMPONENTS		roscpp
					std_msgs
					sensor_msgs
					geometry_msgs
					cv_bridge
					image_transport
					image_geometry
					argus_common
)

message( STATUS ${CMAKE_SOURCE_DIR} )
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake/")

find_package(Boost REQUIRED)

# Find OpenCV3 and set flags for other ROS packages
find_package(OpenCV 3 REQUIRED)
add_definitions( "-DOPENCV3=1" )

find_package(Eigen3 REQUIRED)

find_package(GSL REQUIRED)

catkin_package(
	INCLUDE_DIRS 	include
	LIBRARIES 		odoflow
	CATKIN_DEPENDS 	roscpp 
					image_transport 
					sensor_msgs 
					geometry_msgs 
					cv_bridge 
					std_msgs 
					message_runtime
					argus_common
										
	DEPENDS 		Boost 
					OpenCV 
					Eigen
)

include_directories(include)
include_directories(
	${Boost_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIR}
	${GSL_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "-g -Wall -std=c++11")

if( USE_ROS )
	add_definitions( -DUSE_ROS=1 )
else( USE_ROS )
	add_definitions( -DUSE_ROS=0 )
endif( USE_ROS )

add_library(odoflow
	src/CornerPointDetector.cpp
	src/FASTPointDetector.cpp
	src/FixedPointDetector.cpp
	src/FivePointEstimator.cpp
	src/InterfaceBindings.cpp
	src/InterestPointDetector.cpp
	src/InterestPointTracker.cpp
	src/LKPointTracker.cpp
	src/MotionEstimator.cpp
	src/OpenCVMod.cpp
	src/RigidEstimator.cpp
	src/ROSParser.cpp
	src/VisualOdometryPipeline.cpp
)
# add_dependencies( odoflow ${odoflow_EXPORTED_TARGETS} )
target_link_libraries(odoflow
	${OpenCV_LIBS}
)

add_executable( gsl_test src/GSLTest.cpp )
# add_dependencies( gsl_test ${odoflow_EXPORTED_TARGETS} )
target_link_libraries( gsl_test ${GSL_LIBRARIES} )

add_executable( visual_odometry_node nodes/visual_odometry_node.cpp )
# add_dependencies( visual_odometry_node ${odoflow_EXPORTED_TARGETS} )
target_link_libraries( visual_odometry_node
	odoflow 
	${catkin_LIBRARIES} )

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS odoflow visual_odometry_node
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	FILES_MATCHING PATTERN "*.h"
)
