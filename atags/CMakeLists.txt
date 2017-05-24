cmake_minimum_required(VERSION 2.8.3)
project(atags)

set( CMAKE_CXX_FLAGS "-g -Wall -std=c++0x" )

if( NOT DEFINED CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "" )
    message( STATUS "No build type selected, default to Release" )
    set( CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE )
endif()
message( STATUS "Build configuration: ${CMAKE_BUILD_TYPE}" )

find_package( catkin REQUIRED 
              COMPONENTS cv_bridge
                         image_transport
                         message_generation
                         roscpp
                         std_msgs
                         argus_utils
                         argus_msgs
                         camplex
)

# Add external apriltags library
include( ExternalProject )
ExternalProject_Add( apriltags_external
                     GIT_REPOSITORY   https://github.com/Humhu/apriltags.git
                     BUILD_COMMAND    cmake --build .
                     INSTALL_COMMAND  make install
                     CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CATKIN_DEVEL_PREFIX}
)
set( apriltags_INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include )
set( apriltags_LIBRARIES ${CATKIN_DEVEL_PREFIX}/lib/libapriltags.so )

find_package(Boost REQUIRED)

catkin_package( INCLUDE_DIRS    include
                                ${apriltags_INCLUDE_DIRS}

                LIBRARIES       atags
                                ${apriltags_LIBRARIES}

                CATKIN_DEPENDS  cv_bridge 
                                image_transport 
                                message_runtime 
                                roscpp 
                                std_msgs
                                argus_utils
                                argus_msgs
                                camplex

               DEPENDS          Boost 
                                system_lib
                                apriltags_external
)

include_directories( include
                    ${apriltags_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS}
                    ${catkin_INCLUDE_DIRS}
)

add_library( atags
            src/AtagCommon.cpp
            src/AtagDetector.cpp
)
target_link_libraries( atags
                      ${apriltags_LIBRARIES}
                      ${Boost_LIBRARIES}
                      ${catkin_LIBRARIES}
)
add_dependencies( atags apriltags_external ${catkin_EXPORTED_TARGETS} )

add_executable( detector_node nodes/AtagDetectorNode.cpp )
add_dependencies( detector_node apriltags_external 
)
target_link_libraries( detector_node atags
                                     ${catkin_LIBRARIES}
)

add_executable( gazing_detector_node nodes/GazingAtagDetector.cpp )
add_dependencies( gazing_detector_node apriltags_external 
)
target_link_libraries( gazing_detector_node atags
                                            ${catkin_LIBRARIES}
)

add_executable( syncing_detector_node nodes/SyncingAtagDetector.cpp )
add_dependencies( syncing_detector_node apriltags_external 
)
target_link_libraries( syncing_detector_node atags
                                             ${catkin_LIBRARIES}
)

add_executable( atag_registrar_node nodes/AtagRegistrar.cpp )
target_link_libraries( atag_registrar_node 
        atags 
        ${catkin_LIBRARIES}
)

## Mark executables and/or libraries for installation
install(TARGETS atags
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY include/apriltags/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
)
