#pragma once

#include <yaml-cpp/yaml.h>

#include "extrinsics_array/ExtrinsicsInfoManager.h"

namespace argus
{

/*! \brief Tools for reading/writing extrinsics calibration files.
 * Extrinsics Calibration YAML format:
 * frame_id: [reference frame ID]
 * extrinsics:
 *   position: [x y z]
 *   quaternion: [qw qx qy qz]
 */

/*! \brief Parses a calibration from a YAML object. */
bool ParseExtrinsicsCalibration( const YAML::Node& yaml,
                                 ExtrinsicsInfo& info );

/*! \brief Read a calibration from a file. Returns success. */
bool ReadExtrinsicsCalibration( const std::string& path,
                                ExtrinsicsInfo& info );

/*! \brief Populates a YAML node from an extrinsics calibration. Returns success. */
bool PopulateExtrinsicsCalibration( const ExtrinsicsInfo& info,
                                    YAML::Node& yaml );

/*! \brief Writes a calibration to file. Returns success. */
bool WriteExtrinsicsCalibration( const std::string& path,
                                 const ExtrinsicsInfo& info );

} // end namespace argus
