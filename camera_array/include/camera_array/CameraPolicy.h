#pragma once

namespace camera_array
{

struct RobotState
{
	argus_utils::PoseSE3 pose;
	argus_utils::PoseSE3::CovarianceMatrix poseCovariance;
	argus_utils::PoseSE3::TangentVector velocity;
	argus_utils::PoseSE3::CovarianceMatrix velocityCovariance;	
};

class CameraPolicy
{
public:

	CameraPolicy();
	
	/*! \brief Returns the desired active camera set from a state. */
	virtual CameraSet ComputeAction( const RobotState& robot, 
	                                 const CameraState& camera,
	                                 const FiducialState& fiducials );
	
};
	
}
