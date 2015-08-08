#pragma once

#include <image_geometry/pinhole_camera_model.h>

#include "camera_array/CameraArrayInfo.h"

#include "argus_utils/GeometryUtils.h"

namespace camera_array
{

/*! \brief Models an array of pinhole cameras. */
class PinholeCameraArrayModel
{
public:
	
	typedef std::vector< cv::Point2d > Points;
	
	PinholeCameraArrayModel();
	
	/*! \brief Initialize from an extrinsics message and corresponding
		* intrinsics messages. Note that the ordering must be the same! */
	void FromInfo( const CameraArrayInfo& info );
	
	/*! \brief Projects a point in the array frame onto each cameras image plane. */
	Points Project( const cv::Point3d& point );
	std::vector< Points > Project( const std::vector< cv::Point3d >& points );
	
private:
	
	struct CameraRegistration
	{
		argus_utils::PoseSE3 extrinsic;
		image_geometry::PinholeCameraModel cameraModel;
	};
	
	std::vector< std::string > cameraNames;
	std::vector< CameraRegistration > registry;
	
};
	
}
