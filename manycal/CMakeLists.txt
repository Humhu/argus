cmake_minimum_required(VERSION 2.8.3)
project(manycal)

set(CMAKE_CXX_FLAGS "-g -Wall -std=c++11")

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED 
	COMPONENTS		camera_info_manager
					message_generation
					roscpp
					std_msgs
					sensor_msgs
					visualization_msgs
					argus_utils
					argus_msgs
					image_transport
					cv_bridge
# 					graphopt
					extrinsics_array
					fiducials
)
include_directories( ${catkin_INCLUDE_DIRS} )

# Use Boost for most utilities, threading
find_package(Boost REQUIRED)
include_directories( ${Boost_INCLUDE_DIRS} )

find_package(isam REQUIRED)
include_directories( ${isam_INCLUDE_DIRS} )
message( STATUS ${isam_LIBRARIES} )

add_service_files(
	FILES           CaptureArray.srv
)

generate_messages(
	DEPENDENCIES	std_msgs
)

catkin_package(
	INCLUDE_DIRS 	include
	
	LIBRARIES 		manycal
					
	CATKIN_DEPENDS 	message_runtime 
					roscpp 
					std_msgs
					sensor_msgs
					visualization_msgs
					argus_utils
					argus_msgs
					image_transport
					cv_bridge
# 					graphopt
					extrinsics_array
					fiducials
					
	DEPENDS 		Boost 
					OpenCV
					isam
					system_lib
)

include_directories( include )

add_library( manycal
 	src/ArrayCalibrator.cpp
# 	src/ArrayCapturer.cpp
	src/ArraySynchronizer.cpp
	src/FiducialArrayCalibrator.cpp
	src/ManycalCommon.cpp
	src/sclam_fiducial.cpp
	src/slamse3.cpp
)

# add_dependencies( manycal ${manycal_EXPORTED_TARGETS} )
target_link_libraries( manycal
	${Boost_LIBRARIES}
	${OpenCV_LIBS}
	${isam_LIBRARIES}
	${catkin_LIBRARIES}
)

add_executable( array_calibrator_node nodes/array_calibrator.cpp )
add_dependencies( array_calibrator_node ${PROJECT_NAME}_gencfg )
target_link_libraries( array_calibrator_node 
	${isam_LIBRARIES}
	${catkin_LIBRARIES} 	
	manycal )
	
add_executable( fiducial_array_calibrator_node nodes/fiducial_array_calibrator.cpp )
add_dependencies( fiducial_array_calibrator_node ${PROJECT_NAME}_gencfg )
target_link_libraries( fiducial_array_calibrator_node 
	${isam_LIBRARIES}
	${catkin_LIBRARIES} 	
	manycal )
	
# add_executable( array_capturer_node nodes/array_capture_node.cpp )
# add_dependencies( array_capturer_node ${PROJECT_NAME}_gencfg )
# target_link_libraries( array_capturer_node
# 	${isam_LIBRARIES}
# 	${catkin_LIBRARIES} 	
# 	manycal )

add_executable( isam_test tests/IsamTest.cpp )
add_dependencies( isam_test ${PROJECT_NAME}_gencfg )
target_link_libraries( isam_test
	${isam_LIBRARIES}
	${catkin_LIBRARIES}
	manycal )

add_executable( odometry_test tests/OdometryTest.cpp )
add_dependencies( odometry_test ${PROJECT_NAME}_gencfg )
target_link_libraries( odometry_test
	${isam_LIBRARIES}
	${catkin_LIBRARIES}
	manycal )
	
add_executable( array_synchronizer_node nodes/array_synchronizer_node.cpp )
add_dependencies( array_synchronizer_node ${PROJECT_NAME}_gencfg )
target_link_libraries( array_synchronizer_node
	${isam_LIBRARIES}
	${catkin_LIBRARIES}
	manycal )
	
add_executable( dummy_odometry nodes/dummy_odometry.cpp )
add_dependencies( dummy_odometry ${PROJECT_NAME}_gencfg )
target_link_libraries( dummy_odometry
	${isam_LIBRARIES}
	${catkin_LIBRARIES}
	manycal )
	
add_executable( extrinsics_visualizer nodes/extrinsics_visualizer.cpp )
add_dependencies( extrinsics_visualizer ${PROJECT_NAME}_gencfg )
target_link_libraries( extrinsics_visualizer 
	${catkin_LIBRARIES}
	manycal )
	
## Mark executables and/or libraries for installation
install(TARGETS manycal
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

file(GLOB manycal_HEADERS
	"include/${PROJECT_NAME}/*.h"
	"include/${PROJECT_NAME}/*.hpp"
)
install(FILES ${manycal_HEADERS}
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
