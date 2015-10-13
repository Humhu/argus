#pragma once

#include "argus_utils/PoseSE3.h"
// #include "image_geometry/pinhole_camera_model.h"
#include "camera_array/CameraArrayInfo.h"

#include <unordered_map>

namespace camera_array
{

class CameraArray 
{
public:
	
	CameraArray();
	
	/*! \brief Populates the array using a camera array calibration message. */
	void FromInfo( const CameraArrayInfo& info );
	// TODO CameraArrayInfo ToInfo() const
	
	/*! \brief Get the pose of a camera relative to the array reference frame. */
	argus_utils::PoseSE3 GetCameraExtrinsics( const std::string& name ) const;
	
// 	/*! \brief Get the pinhole model of a camera in the array. */
// 	image_geometry::PinholeCameraModel GetCameraIntrinsics( const std::string& name ) const;
	
	/*! \brief Returns an array object containing a subset of this array. */
	CameraArray GetSubset( const std::vector< std::string >& names ) const;
	
private:
	
	struct CameraRegistration
	{
		argus_utils::PoseSE3 extrinsics;
// 		image_geometry::PinholeCameraModel cameraModel;
	};
	
	std::unordered_map< std::string, CameraRegistration > cameraRegistry;
	
};
	
} // end namespace camera_array
