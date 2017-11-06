#include "manycal/Factors.h"
#include <sstream>

namespace argus
{
isam::FiducialFactor::Ptr create_fiducial_factor( const ros::Time& time,
                                                  const FiducialDetection& detection,
                                                  const MatrixType& cov,
                                                  CameraRegistration& camera,
                                                  FiducialRegistration& fiducial )
{
	isam::FiducialFactor::Properties props;
	props.optCamReference = camera.parent.IsPoseOptimizing();
	props.optCamIntrinsics = camera.IsIntrinsicsOptimizing();
	props.optCamExtrinsics = camera.IsExtrinsicsOptimizing();
	props.optFidReference = fiducial.parent.IsPoseOptimizing();
	props.optFidIntrinsics = fiducial.IsIntrinsicsOptimizing();
	props.optFidExtrinsics = fiducial.IsExtrinsicsOptimizing();

	if( cov.rows() != 2 * detection.points.size() || cov.cols() != 2 * detection.points.size() )
	{
		std::stringstream ss;
		ss << "Got " << cov.rows() << " x " << cov.cols() << " covariance but has "
		   << detection.points.size() << " points!";
		throw std::invalid_argument( ss.str() );
	}

	isam::PoseSE3_Node* camParentNode = camera.parent.GetPoseNode( time );	
	isam::PoseSE3_Node* fidParentNode = fiducial.parent.GetPoseNode( time );
	if( !camParentNode || !fidParentNode )
	{
		return nullptr;
	}

	return std::make_shared<isam::FiducialFactor>( camParentNode,
	                                               camera.GetIntrinsicsNode(),
	                                               camera.GetExtrinsicsNode(),
	                                               fidParentNode,
	                                               fiducial.GetIntrinsicsNode(),
	                                               fiducial.GetExtrinsicsNode(),
	                                               DetectionToIsam( detection ),
	                                               isam::Covariance( cov ),
	                                               props );
}
}