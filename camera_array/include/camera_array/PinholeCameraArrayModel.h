#pragma once

#include "camera_array/CameraArrayInfo.h"

#include "camplex/CameraCalibration.h"

#include "argus_utils/GeometryUtils.h"

namespace camera_array
{

struct CameraObservation
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	
	bool valid;
	Eigen::Vector2d imageCoordinates;
	
};
	
/*! \brief Models an array of pinhole cameras. */
class PinholeCameraArrayModel
{
public:
	
	typedef std::vector< cv::Point2d > Points;
	
	PinholeCameraArrayModel();
	
	/*! \brief Initialize from a camera array calibration. */
	void FromInfo( const CameraArrayInfo& info );
	
	/*! \brief Projects a point [x,y,z] to image coordinates for each camera. */
	std::vector<CameraObservation> Project( const Eigen::Vector3d& point );
	
	/*! \brief Returns the image bounds for each camera. */
	std::vector< cv::Rect > GetRois() const;
	
private:
	
	struct CameraRegistration
	{
		argus_utils::PoseSE3 extrinsicInverse;
		camplex::CameraCalibration intrinsic;
	};
	
	std::vector< std::string > cameraNames;
	std::vector< CameraRegistration > registry;
	
	static Eigen::Vector2d ProjectPoint( const camplex::CameraCalibration& intrinsic,
										 const Eigen::Vector3d& point );
	
};

}
