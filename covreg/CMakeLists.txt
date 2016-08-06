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
					lookup
					broadcast
					visualization_msgs
)
include_directories( ${catkin_INCLUDE_DIRS} )

# Use Boost for most utilities, threading
find_package(Boost REQUIRED)
include_directories( ${Boost_INCLUDE_DIRS} )
list( APPEND covreg_LIBS ${Boost_LIBRARIES} )


find_package(percepto REQUIRED)
message( STATUS "percepto found at: " ${percepto_INCLUDE_DIRS} )
include_directories( ${percepto_INCLUDE_DIRS} )
list( APPEND covreg_LIBS ${percepto_LIBS} )

find_package(Eigen3 REQUIRED)
include_directories( ${EIGEN3_INCLUDE_DIR} )

add_message_files( FILES
	CovarianceEstimatorInfo.msg
	NeuralNetworkInfo.msg
	OptimizerStatus.msg
)

add_service_files( FILES
	QueryCovariance.srv 
)

generate_messages(
	DEPENDENCIES	std_msgs
	            	argus_msgs
)

catkin_package(
	INCLUDE_DIRS	include
	
	LIBRARIES		covreg
					
	CATKIN_DEPENDS  message_runtime 
					roscpp 
					std_msgs
					sensor_msgs
					argus_utils
					argus_msgs
					lookup
					broadcast
					visualization_msgs
					
	DEPENDS 		Boost 
					percepto
					Eigen3
					system_lib
)

include_directories( include )

add_library( covreg
	src/AdaptiveCovarianceEstimator.cpp
	src/ClipOptimizer.cpp
	src/CovarianceEstimator.cpp
	src/CovarianceManager.cpp
	src/EstimatorInfoParsers.cpp
	src/InnovationLikelihoodProblem.cpp
	src/KalmanFilterEpisode.cpp
	src/KalmanFilterModules.cpp
	src/PositiveDefiniteModules.cpp
)
add_dependencies( covreg ${covreg_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
add_dependencies( covreg covreg_gencfg )
target_link_libraries( covreg
	${catkin_LIBRARIES}
	${covreg_LIBS}
)

add_executable( optimizer_node nodes/covariance_optimizer.cpp )
add_dependencies( optimizer_node covreg_gencfg )
target_link_libraries( optimizer_node
                       ${catkin_LIBRARIES}
                       ${covreg_LIBS}
                       covreg )

add_executable( covariance_repeater nodes/covariance_repeater.cpp )
add_dependencies( covariance_repeater covreg_gencfg )
target_link_libraries( covariance_repeater
                       ${catkin_LIBRARIES}
                       ${covreg_LIBS}
                       covreg )

add_executable( performance_estimator nodes/performance_estimator.cpp )
add_dependencies( performance_estimator covreg_gencfg )
target_link_libraries( performance_estimator
                       ${catkin_LIBRARIES}
                       ${covreg_LIBS}
                       covreg )

add_executable( adaptive_covariance_repeater nodes/adaptive_covariance_repeater.cpp )
add_dependencies( adaptive_covariance_repeater covreg_gencfg )
target_link_libraries( adaptive_covariance_repeater
                       ${catkin_LIBRARIES}
                       ${covreg_LIBS}
                       covreg )

add_executable( velocity_visualizer nodes/velocity_visualizer.cpp )
add_dependencies( velocity_visualizer covreg_gencfg )
target_link_libraries( velocity_visualizer
                       ${catkin_LIBRARIES}
                       ${covreg_LIBS}
                       covreg )

## Mark executables and/or libraries for installation
install(TARGETS covreg optimizer_node covariance_repeater performance_estimator adaptive_covariance_repeater
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

file(GLOB covreg_HEADERS
	"include/covreg/*.h"
	"include/covreg/*.hpp"
)
install(FILES ${covreg_HEADERS}
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
