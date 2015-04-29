###############################################################################
# Find Eigen3
#
# This sets the following variables:
# Eigen_FOUND - True if Eigen was found.
# Eigen_INCLUDE_DIRS - Directories containing the Eigen include files.
# Eigen_DEFINITIONS - Compiler flags for Eigen.

find_package(PkgConfig QUIET)
pkg_check_modules(PC_Eigen eigen3)
set(Eigen_DEFINITIONS ${PC_Eigen_CFLAGS_OTHER})

if(CMAKE_SYSTEM_NAME STREQUAL Linux)
    set(CMAKE_INCLUDE_PATH ${CMAKE_INCLUDE_PATH} /usr /usr/local)
endif(CMAKE_SYSTEM_NAME STREQUAL Linux)

find_path(Eigen_INCLUDE_DIR Eigen/Core
    HINTS ${PC_Eigen_INCLUDEDIR} ${PC_Eigen_INCLUDE_DIRS} "${Eigen_ROOT}" "$ENV{Eigen_ROOT}"
    PATHS "$ENV{PROGRAMFILES}/Eigen" "$ENV{PROGRAMW6432}/Eigen"
          "$ENV{PROGRAMFILES}/Eigen 3.0.0" "$ENV{PROGRAMW6432}/Eigen 3.0.0"
    PATH_SUFFIXES eigen3 include/eigen3 include)


set(Eigen_INCLUDE_DIRS ${Eigen_INCLUDE_DIR})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Eigen DEFAULT_MSG Eigen_INCLUDE_DIR)

mark_as_advanced(Eigen_INCLUDE_DIR)

if(Eigen_FOUND)
  message(STATUS "Eigen found (include: ${Eigen_INCLUDE_DIRS})")
endif(Eigen_FOUND)

