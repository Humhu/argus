#pragma once

#include "camera_array/CameraArrayInfo.h"

namespace camera_array
{

// TODO Distortion parameters
/*! \brief Tools for reading/writing extrinsics calibration files.
 * CameraArray Calibration YAML format:
 * array_name: [reference frame name]
 * cameras:
 *   [object1 name]:
 *     extrinsics:
 *       position: [x y z]
 *       quaternion: [qw qx qy qz]
 *   [object2 name]:
 *     etc...
 */

/*! \brief Read a calibration from a file. Returns success. */
bool ReadCameraArrayCalibration( const std::string& path,
                                 std::string& refName,
                                 CameraArrayInfo& info );
	
/*! \brief Writes a calibration to file. Returns success. */
bool WriteCameraArrayCalibration( const std::string& path,
                                  const std::string& refName,
                                  const CameraArrayInfo& info );
	
}
