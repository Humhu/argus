cmake_minimum_required(VERSION 2.8.3)
project(broadcast)

set(CMAKE_CXX_FLAGS "-g -Wall -std=c++11")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake/")

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
             message_generation
             std_msgs
             argus_msgs
             nav_msgs
             roscpp
             argus_utils
             lookup
)

# Use Boost for most utilities, threading
find_package(Boost REQUIRED)

# Use Eigen for matrices, linear algebra
find_package(Eigen3 REQUIRED)

add_service_files( FILES
                   QueryFeatures.srv )

catkin_python_setup()

generate_messages( DEPENDENCIES
                  std_msgs 
                  argus_msgs )

catkin_package( INCLUDE_DIRS     include

                LIBRARIES        broadcast

                CATKIN_DEPENDS   message_runtime
                                 std_msgs
                                 argus_msgs
                                 nav_msgs
                                 roscpp 
                                 argus_utils
                                 lookup

                DEPENDS          Boost 
                                 system_lib
)

include_directories( include
                    ${Boost_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIR}
                    ${catkin_INCLUDE_DIRS}
                    )

add_library( broadcast
            src/BroadcastCommon.cpp
            src/BroadcastInfoManager.cpp
            src/BroadcastMultiReceiver.cpp
            src/BroadcastTransmitter.cpp
            src/BroadcastReceiver.cpp
            )
target_link_libraries( broadcast
                      ${Boost_LIBRARIES}
                      ${catkin_LIBRARIES}
                      )
add_dependencies( broadcast ${broadcast_EXPORTED_TARGETS} )

add_executable( belief_state_broadcaster nodes/belief_state_broadcaster.cpp )
add_dependencies( belief_state_broadcaster covreg_gencfg ${broadcast_EXPORTED_TARGETS} )
target_link_libraries( belief_state_broadcaster ${catkin_LIBRARIES} broadcast )

add_executable( broadcast_listener nodes/broadcast_listener.cpp )
add_dependencies( broadcast_listener covreg_gencfg ${broadcast_EXPORTED_TARGETS} )
target_link_libraries( broadcast_listener ${catkin_LIBRARIES} broadcast )


## Mark executables and/or libraries for installation
install(TARGETS broadcast belief_state_broadcaster broadcast_listener
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        )
