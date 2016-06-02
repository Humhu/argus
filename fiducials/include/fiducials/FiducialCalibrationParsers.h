#pragma once

#include <yaml-cpp/yaml.h>

#include "fiducials/FiducialInfo.h"

namespace argus
{

/*! \brief Tools for reading/writing fiducial calibration files.
 * 
 * Fiducial Calibration YAML format:
 * 
 * intrinsics:
 *   points_x: [x0, x1, ...]
 *   points_y: [y0, y1, ...]
 *   points_z: [z0, z1, ...]
 */

/*! \brief Parses a calibration from a YAML object. Returns success. */
bool ParseFiducialCalibration( const YAML::Node& yaml, fiducials::FiducialInfo& info );

/*! \brief Reads a fiducial calibration from a YAML file. Returns success. */
bool ReadFiducialCalibration( const std::string& path, fiducials::FiducialInfo& info );

/*! \brief Populates a YAML node from a fiducial calibration. */
void PopulateFiducialCalibration( const fiducials::FiducialInfo& info, YAML::Node& yaml );

/*! \brief Writes a fiducial calibration to a YAML file. Returns success. */
bool WriteFiducialCalibration( const std::string& path, const fiducials::FiducialInfo& info );

} // end namespace argus
