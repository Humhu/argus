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
					argus_utils
					camplex
					paraset
)

message( STATUS ${CMAKE_SOURCE_DIR} )
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake/")

find_package(Boost REQUIRED)

# Find OpenCV3 and set flags for other ROS packages
find_package(OpenCV 3 REQUIRED)
add_definitions( "-DOPENCV3=1" )

find_package(GSL REQUIRED)

catkin_package(
	INCLUDE_DIRS 	include
	LIBRARIES 		odoflow
	CATKIN_DEPENDS 	roscpp 
					sensor_msgs 
					geometry_msgs 
					cv_bridge 
					image_transport 
					std_msgs 
					message_runtime
					argus_utils
					camplex
					paraset
					
	DEPENDS 		Boost 
					OpenCV 
)

include_directories(include)
include_directories(
	${Boost_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${GSL_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "-g -Wall -std=c++0x")

add_library( odoflow
	src/CornerPointDetector.cpp
	src/ECCDenseTracker.cpp
	src/FASTPointDetector.cpp
	src/FixedPointDetector.cpp
	src/LKPointTracker.cpp
	src/MotionPredictor.cpp
	src/OdoflowCommon.cpp
	src/RigidEstimator.cpp
	src/VelocityPublisher.cpp
)
target_link_libraries( odoflow
	${OpenCV_LIBS}
	${catkin_LIBRARIES}
)

add_executable( sparse_vo_node nodes/sparse_vo_node.cpp )
target_link_libraries( sparse_vo_node
					   odoflow 
					   ${catkin_LIBRARIES} )

add_executable( dense_vo_node nodes/dense_vo_node.cpp )
target_link_libraries( dense_vo_node
					   odoflow 
					   ${catkin_LIBRARIES} )

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS odoflow sparse_vo_node dense_vo_node
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	FILES_MATCHING PATTERN "*.h"
)
