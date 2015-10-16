#pragma once

#include <yaml-cpp/yaml.h>

#include "extrinsics_array/ExtrinsicsArrayInfo.h"

namespace extrinsics_array
{

/*! \brief Tools for reading/writing extrinsics calibration files.
 * Array Calibration YAML format:
 * frame_id: [reference frame ID]
 * [member1 name]:
 *   extrinsics:
 *     position: [x y z]
 *     quaternion: [qw qx qy qz]
 * [member2 name]:
 *   etc...
 */

/*! \brief Parses a calibration from a YAML object. */
bool ParseExtrinsicsArrayCalibration( const YAML::Node& yaml,
                                      ExtrinsicsArrayInfo& info );

/*! \brief Read a calibration from a file. Returns success. */
bool ReadExtrinsicsArrayCalibration( const std::string& path,
                                     ExtrinsicsArrayInfo& info );

/*! \brief Populates a YAML node from an extrinsics calibration. Returns success. */
bool PopulateExtrinsicsArrayCalibration( const ExtrinsicsArrayInfo& info,
                                         YAML::Node& yaml );

/*! \brief Writes a calibration to file. Returns success. */
bool WriteExtrinsicsArrayCalibration( const std::string& path,
                                      const ExtrinsicsArrayInfo& info );

} // end namespace fiducial_array
