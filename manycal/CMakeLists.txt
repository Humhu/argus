cmake_minimum_required(VERSION 2.8.3)
project(manycal)

set(CMAKE_CXX_FLAGS "-g -Wall -std=c++11")

if( NOT DEFINED CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "" )
    message( STATUS "No build type selected, default to Release" )
    set( CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE )
endif()
message( STATUS "Build configuration: ${CMAKE_BUILD_TYPE}" )

find_package( catkin REQUIRED 
              COMPONENTS message_generation
                         roscpp
                         std_msgs
                         geometry_msgs
                         sensor_msgs
                         visualization_msgs
                         argus_utils
                         argus_msgs
                         graphopt
                         extrinsics_array
                         fiducials
)

# Use Boost for most utilities, threading
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)

add_service_files( FILES CaptureArray.srv
                         SetThrottleWeight.srv
                         WriteCalibration.srv
)

generate_messages( DEPENDENCIES std_msgs
                                geometry_msgs
)

catkin_package( INCLUDE_DIRS    include

                LIBRARIES       manycal

                CATKIN_DEPENDS  message_runtime 
                                roscpp 
                                std_msgs
                                geometry_msgs
                                sensor_msgs
                                visualization_msgs                                
                                argus_utils
                                argus_msgs
                                graphopt
                                extrinsics_array
                                fiducials

               DEPENDS          Boost 
                                OpenCV
                                system_lib
)

include_directories( include 
                    ${EIGEN3_INCLUDE_DIR}
                    ${Boost_INCLUDE_DIRS}
                    ${catkin_INCLUDE_DIRS}
)

add_library( manycal
             #src/ArrayCalibrator.cpp
             src/CameraArrayCalibrator.cpp
             src/FiducialArrayCalibrator.cpp
             src/ManycalVisualization.cpp
             src/ManycalCommon.cpp
             src/sclam_fiducial.cpp
             src/slamse3.cpp
)
add_dependencies( manycal ${manycal_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
target_link_libraries( manycal
                       ${Boost_LIBRARIES}
                       ${OpenCV_LIBS}
                       ${isam_LIBRARIES}
                       ${catkin_LIBRARIES}
)

# add_executable( array_calibrator_node nodes/array_calibrator.cpp )
# add_dependencies( array_calibrator_node ${PROJECT_NAME}_gencfg )
# target_link_libraries( array_calibrator_node 
#                        ${isam_LIBRARIES}
#                        ${catkin_LIBRARIES}
#                        manycal 
# )

add_executable( fiducial_array_calibrator_node nodes/fiducial_array_calibrator.cpp )
add_dependencies( fiducial_array_calibrator_node ${PROJECT_NAME}_gencfg )
target_link_libraries( fiducial_array_calibrator_node 
                       ${isam_LIBRARIES}
                       ${catkin_LIBRARIES}
                       manycal
)

add_executable( camera_array_calibrator_node nodes/camera_array_calibrator.cpp )
add_dependencies( camera_array_calibrator_node ${PROJECT_NAME}_gencfg )
target_link_libraries( camera_array_calibrator_node 
                       ${isam_LIBRARIES}
                       ${catkin_LIBRARIES}
                       manycal 
)

add_executable( isam_test tests/IsamTest.cpp )
add_dependencies( isam_test ${PROJECT_NAME}_gencfg )
target_link_libraries( isam_test
                       ${isam_LIBRARIES}
                       ${catkin_LIBRARIES}
                       manycal
)

add_executable( odometry_test tests/OdometryTest.cpp )
add_dependencies( odometry_test ${PROJECT_NAME}_gencfg )
target_link_libraries( odometry_test
                      ${isam_LIBRARIES}
                      ${catkin_LIBRARIES}
                      manycal
)

add_executable( cam_synchronizer nodes/camera_synchronizer.cpp )
add_dependencies( cam_synchronizer ${PROJECT_NAME}_gencfg )
target_link_libraries( cam_synchronizer
                       ${isam_LIBRARIES}
                       ${catkin_LIBRARIES}
                       manycal
)

add_executable( cam_throttler nodes/camera_throttler.cpp )
add_dependencies( cam_throttler ${PROJECT_NAME}_gencfg )
target_link_libraries( cam_throttler
                       ${isam_LIBRARIES}
                       ${catkin_LIBRARIES}
                       manycal
)

## Mark executables and/or libraries for installation
install(TARGETS manycal
                fiducial_array_calibrator_node
                camera_array_calibrator_node
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
