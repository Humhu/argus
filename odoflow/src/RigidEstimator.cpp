#include "odoflow/RigidEstimator.h"
#include "odoflow/OpenCVMod.h"

using namespace argus_utils;

namespace odoflow
{

	RigidEstimator::RigidEstimator( ros::NodeHandle& nh, ros::NodeHandle& ph )
	: MotionEstimator( nh, ph )
	{
	  ph.param<double>( "estimator/scale", scale, 1.0 );
	}
	
	bool RigidEstimator::EstimateMotion( const InterestPoints& srcPoints,
										 const InterestPoints& dstPoints,
										 PoseSE3& transform )
	{
		
		cv::Mat Hest = cv::estimateRigidTransform( srcPoints,
												   dstPoints, false );
		
		if( Hest.empty() )
		{
			return false;
		}
		
		Eigen::Matrix<double,2,3> Ab = MatToEigen<double,2,3>( Hest );
		Eigen::Matrix<double,2,2> A = Ab.block<2,2>(0,0);
		double zScale = A.determinant();
		Ab.block<2,2>(0,0) = A/zScale;
		
		Eigen::Matrix<double,4,4> H = Eigen::Matrix<double,4,4>::Identity();
		H.block<2,2>(1,1) = Ab.block<2,2>(0,0);
		H(1,3) = -Ab(0,2) * scale; // Image x corresponds to camera -y
		H(2,3) = -Ab(1,2) * scale; // Image y corresponds to camera -z
		
		transform = PoseSE3(H);
		return true;
		
	}
	
}
