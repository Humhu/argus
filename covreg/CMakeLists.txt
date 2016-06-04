cmake_minimum_required(VERSION 2.8.3)
project(covreg)

set(CMAKE_CXX_FLAGS "-g -Wall -std=c++11")

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED 
	COMPONENTS		message_generation
					roscpp
					std_msgs
					sensor_msgs
					argus_utils
					argus_msgs
)
include_directories( ${catkin_INCLUDE_DIRS} )

# Use Boost for most utilities, threading
find_package(Boost REQUIRED)
include_directories( ${Boost_INCLUDE_DIRS} )
list( APPEND covreg_LIBRARIES ${Boost_LIBRARIES} )


find_package(percepto REQUIRED)
message( STATUS "percepto found at: " ${percepto_INCLUDE_DIRS} )
include_directories( ${percepto_INCLUDE_DIRS} )
list( APPEND covreg_LIBRARIES ${percepto_LIBS} )

find_package(Eigen3 REQUIRED)
include_directories( ${EIGEN3_INCLUDE_DIR} )

# add_service_files(
# 	FILES           CaptureArray.srv
# 	                WriteCalibration.srv
# )

# generate_messages(
# 	DEPENDENCIES	std_msgs
# )

catkin_package(
	INCLUDE_DIRS 	include
	
	LIBRARIES 		covreg
					
	CATKIN_DEPENDS 	message_runtime 
					roscpp 
					std_msgs
					sensor_msgs
					argus_utils
					argus_msgs
					
	DEPENDS 		Boost 
					percepto
					Eigen3
					system_lib
)

include_directories( include )

add_library( covreg
	src/PerceptoInterface.cpp
)
#add_dependencies( covreg ${covreg_EXPORTED_TARGETS} )
target_link_libraries( covreg
	${catkin_LIBRARIES}
	${covreg_LIBRARIES}
)

add_executable( test_prediction tests/TestPrediction.cpp )
add_dependencies( test_prediction ${PROJECT_NAME}_gencfg )
target_link_libraries( test_prediction
                       ${catkin_LIBRARIES}
                       ${covreg_LIBRARIES}
                       covreg )

## Mark executables and/or libraries for installation
install(TARGETS test_prediction
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

file(GLOB covreg_HEADERS
	"include/${PROJECT_NAME}/*.h"
	"include/${PROJECT_NAME}/*.hpp"
)
install(FILES ${covreg_HEADERS}
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
