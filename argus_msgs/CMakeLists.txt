cmake_minimum_required(VERSION 2.8.3)
project(argus_msgs)

find_package(catkin REQUIRED 
	COMPONENTS		message_generation
					std_msgs
					sensor_msgs
)

## Generate service files
add_message_files(
	FILES		FiducialDetection.msg
				ImageFiducialDetections.msg
				Point2D.msg
				Point2D_32.msg 
				TagDetection.msg 
				TagDetectionsStamped.msg
)

generate_messages(
	DEPENDENCIES 	std_msgs
					sensor_msgs
)

catkin_package(
	CATKIN_DEPENDS 	message_runtime 
					std_msgs 
)
