#pragma once

#include "extrinsics_info_manager/ExtrinsicsInfo.h"

namespace extrinsics_info_manager
{
	
/*! \brief Tools for reading/writing extrinsics calibration files.
 * Extrinsics Calibration YAML format:
 * reference_name: [reference frame name]
 * frames:
 *   [object1 name]:
 *     position: [x y z]
 *     quaternion: [qw qx qy qz]
 *   [object2 name]:
 *     etc...
 */

/*! \brief Read a calibration from a file. Returns success. */
bool ReadExtrinsicsCalibration( const std::string& path,
								std::string& refName,
								ExtrinsicsInfo& info );
	
/*! \brief Writes a calibration to file. Returns success. */
bool WriteExtrinsicsCalibration( const std::string& path,
								 const std::string& refName,
								 const ExtrinsicsInfo& info );
	
}
