#pragma once

#include <yaml-cpp/yaml.h>

#include "fiducial_array/FiducialInfo.h"
#include "fiducial_array/FiducialArrayInfo.h"

namespace fiducial_array
{

/*! \brief Tools for reading/writing fiducial calibration files.
 * 
 * Fiducial Calibration YAML format:
 * intrinsics:
 *   points_x: [x0, x1, ...]
 *   points_y: [y0, y1, ...]
 *   points_z: [z0, z1, ...]
 */

/*! \brief Parses a calibration from a YAML object. Returns success. */
bool ParseFiducialCalibration( const YAML::Node& yaml, FiducialInfo& info );

/*! \brief Reads a fiducial calibration from a YAML file. Returns success. */
bool ReadFiducialCalibration( const std::string& path, FiducialInfo& info );

/*! \brief Populates a YAML node from a fiducial calibration. Returns success. */
bool PopulateFiducialCalibration( const FiducialInfo& info, YAML::Node& yaml );

/*! \brief Writes a fiducial calibration to a YAML file. Returns success. */
bool WriteFiducialCalibration( const std::string& path, const FiducialInfo& info );


/*! \brief Parse a fiducial array calibration from a YAML object. */
bool ParseFiducialArrayCalibration( const YAML::Node& yaml, FiducialArrayInfo& info );

/*! \brief Reads a fiducial array calibration from a YAML file. Returns success. */
bool ReadFiducialArrayCalibration( const std::string& path, FiducialArrayInfo& info );

/*! \brief Populates a YAML node from a fiducial array calibration. Returns succes. */
bool PopulateFiducialArrayCalibration( const FiducialArrayInfo& info, YAML::Node& yaml );

/*! \brief Writes a fiducial array calibraiton to a YAML file. Returns success. */
bool WriteFiducialArrayCalibration( const std::string& path, const FiducialArrayInfo& info );

} // end namespace fiducial_array
