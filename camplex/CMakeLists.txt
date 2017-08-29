cmake_minimum_required(VERSION 2.8.3)
project(camplex)

set(CMAKE_CXX_FLAGS "-g -Wall -std=c++0x")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake/")

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED 
	COMPONENTS		camera_calibration_parsers
					camera_info_manager
					cv_bridge
					image_transport
					message_generation
					roscpp
					std_msgs
					sensor_msgs
					geometry_msgs
					argus_utils
					lookup
					extrinsics_array
					vizard
					paraset
)

# Use Boost for most utilities, threading
find_package(Boost REQUIRED)

# Find OpenCV and set flag for OpenCV3 if needed
find_package(OpenCV 3 REQUIRED)
message( STATUS ${OpenCV_INCLUDE_DIRS} )
message( STATUS ${OpenCV_LIBS} )
add_definitions("-DOPENCV3=1")

# Use userspace libv4l2 for interfacing
find_package(V4L2 REQUIRED)

add_message_files(
	FILES			FiducialInfo.msg
)

add_service_files( 
	FILES 			CaptureFrames.srv
					GetCameraInfo.srv
					PrintCapabilities.srv
					SetStreaming.srv
)

generate_messages(
	DEPENDENCIES	sensor_msgs
					std_msgs
					geometry_msgs
)

catkin_package(
	INCLUDE_DIRS 	include
	
	LIBRARIES 		camplex
					
	CATKIN_DEPENDS 	camera_calibration_parsers
					camera_info_manager
					cv_bridge 
					image_transport 
					message_runtime 
					roscpp 
					std_msgs
					sensor_msgs 
					geometry_msgs
					argus_utils
					lookup
					extrinsics_array
					vizard
					paraset
					
	DEPENDS 		Boost 
					OpenCV 
					system_lib
)

include_directories( include
					 ${Boost_INCLUDE_DIRS}
					 ${OpenCV_INCLUDE_DIRS}
					 ${V4L2_INCLUDE_DIRS}
					 ${catkin_INCLUDE_DIRS}
)

add_library( camplex
	src/CameraCalibration.cpp
	# src/CameraCalibrator.cpp
	src/CameraDriver.cpp
	src/CamplexCommon.cpp
	src/DriverNode.cpp
	src/FiducialArray.cpp
	src/FiducialCalibrationParsers.cpp
	src/FiducialInfoManager.cpp
	src/FiducialCommon.cpp
	src/FiducialVisualizer.cpp
	src/SplitStereoDriverNode.cpp
)
add_dependencies( camplex ${camplex_EXPORTED_TARGETS})
target_link_libraries( camplex
	${Boost_LIBRARIES}
	${OpenCV_LIBS}
	${V4L2_LIBRARIES}
	${catkin_LIBRARIES}
)

add_executable( camera_node
	nodes/camera_node.cpp )
target_link_libraries( camera_node
	${catkin_LIBRARIES}
	camplex
	${OpenCV_LIBS} )

add_executable( split_camera_node
	nodes/split_camera_node.cpp )
target_link_libraries( split_camera_node
	${catkin_LIBRARIES}
	camplex
	${OpenCV_LIBS} )

add_executable( viewer_node 
	nodes/viewer_node.cpp )
target_link_libraries( viewer_node
	${catkin_LIBRARIES}
	${OpenCV_LIBS} )
	
add_executable( recorder_node 
	nodes/recorder_node.cpp )
target_link_libraries( recorder_node
	${catkin_LIBRARIES}
	${OpenCV_LIBS} )
	
add_executable( video_recorder_node 
	nodes/video_recorder.cpp )
target_link_libraries( video_recorder_node
	${catkin_LIBRARIES}
	${OpenCV_LIBS} )

add_executable( undistortion_node
	nodes/undistortion_node.cpp )
target_link_libraries( undistortion_node
    camplex
	${catkin_LIBRARIES}
	${OpenCV_LIBS} )

add_executable( fiducial_pose_estimator
	nodes/fiducial_pose_estimator.cpp )
target_link_libraries( fiducial_pose_estimator
	camplex
	${catkin_LIBRARIES} )

add_executable( checkerboard_registrar_node
	nodes/checkerboard_registrar.cpp )
target_link_libraries( checkerboard_registrar_node
	camplex ${catkin_LIBRARIES} )

add_executable( checkerboard_detector_node
	nodes/checkerboard_detector_node.cpp )
target_link_libraries( checkerboard_detector_node
	camplex ${catkin_LIBRARIES} )

add_executable( resize_node nodes/resize_node.cpp )
target_link_libraries( resize_node camplex ${catkin_LIBRARIES} )
	
## Mark executables and/or libraries for installation
install(TARGETS camplex camera_node viewer_node recorder_node video_recorder_node undistortion_node
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/camplex/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
	FILES_MATCHING PATTERN "*.h"
)
