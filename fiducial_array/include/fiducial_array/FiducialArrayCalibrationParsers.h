#pragma once

#include <yaml-cpp/yaml.h>
#include "fiducial_array/FiducialArrayInfo.h"

namespace fiducial_array
{

// TODO Distortion parameters
/*! \brief Tools for reading/writing extrinsics calibration files.
 * FiducialArray Calibration YAML format:
 * array_name: [reference frame name]
 * fiducials:
 *   [object1 name]:
 *     extrinsics:
 *       position: [x y z]
 *       quaternion: [qw qx qy qz]
 *     intrinsics:
 *       points_x: [x1 x2 ...]
 *       points_y: [y1 y2 ...]
 *       points_z: [z1 z2 ...]
 *   [object2 name]:
 *     etc...
 */

/*! \brief Parses a calibration from a YAML object returned by the ROS parameter
 * server. */
bool ParseFiducialArrayCalibration( const YAML::Node& yaml,
									std::string& refName,
									FiducialArrayInfo& info );

/*! \brief Read a calibration from a file. Returns success. */
bool ReadFiducialArrayCalibration( const std::string& path,
                                   std::string& refName,
                                   FiducialArrayInfo& info );
	
/*! \brief Writes a calibration to file. Returns success. */
bool WriteFiducialArrayCalibration( const std::string& path,
                                    const std::string& refName,
                                    const FiducialArrayInfo& info );
	
} // end namespace fiducial_array
