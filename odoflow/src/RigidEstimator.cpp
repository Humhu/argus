#include "odoflow/RigidEstimator.h"
#include "odoflow/OpenCVMod.h"

#include "argus_utils/utils/ParamUtils.h"

#include <Eigen/SVD>

namespace argus
{

RigidEstimator::RigidEstimator( ros::NodeHandle& nh, ros::NodeHandle& ph )
: MotionEstimator( nh, ph )
{
	GetParam( ph, "scale", _scale );

	FullNumericRange reprojThreshold;
	GetParam( ph, "reprojection_threshold", reprojThreshold);
	_reprojThreshold.Initialize( ph, reprojThreshold.init, "reprojection_threshold", 
	                             "RANSAC reprojection inlier threshold" );
	_reprojThreshold.AddCheck<GreaterThanOrEqual>( reprojThreshold.min );
	_reprojThreshold.AddCheck<LessThanOrEqual>( reprojThreshold.max );

	FullNumericRange maxIters;
	GetParam( ph, "max_iters", maxIters );
	_maxIters.Initialize( ph, maxIters.init, "max_iters",
	                      "RANSAC max iterations" );
	_maxIters.AddCheck<GreaterThanOrEqual>( maxIters.min );
	_maxIters.AddCheck<LessThanOrEqual>( maxIters.max );
	_maxIters.AddCheck<IntegerValued>( ROUND_CEIL );
}

bool RigidEstimator::EstimateMotion( const InterestPoints& srcPoints,
                                     const InterestPoints& dstPoints,
                                     std::vector<uchar>& inliers,
                                     PoseSE3& transform,
                                     PoseSE2& frameTransform )
{
	if( srcPoints.empty() || dstPoints.empty() )
	{
		return false;
	}

	cv::Mat Hxest = cv::findHomography( srcPoints, dstPoints, cv::RANSAC, 
	                                    _reprojThreshold, inliers, _maxIters );
	if( Hxest.empty() )
	{
		return false;
	}

	InterestPoints srcInliers, dstInliers;
	for( unsigned int i = 0; i < srcPoints.size(); i++ )
	  {
	    if( inliers[i] ) 
	      {
		srcInliers.push_back( srcPoints[i] );
		dstInliers.push_back( dstPoints[i] );
	      }
	  }

	cv::Mat Hest = cv::estimateRigidTransform( srcInliers, dstInliers, false );
	if( Hest.size().height == 0 || Hest.size().width == 0 )
	{
		ROS_WARN_STREAM( "RigidEstimator: Could not estimate motion." );
		return false;
	}

	Eigen::Matrix<double,2,3> Ab = MatToEigen<double,2,3>( Hest );
	
	// Extract the rotation using Procrustes solution
	Eigen::Matrix2d A = Ab.block<2,2>(0,0);
	Eigen::JacobiSVD<Eigen::Matrix2d> svd( A, Eigen::ComputeFullU | Eigen::ComputeFullV );
	Eigen::Matrix2d R = svd.matrixU() * svd.matrixV().transpose();
	
	// ROS_INFO_STREAM( "Ab: " << std::endl << Ab );
	// ROS_INFO_STREAM( "R: " << std::endl << R );

	MatrixType h = MatrixType::Identity(3,3);
	h.block<2,2>(0,0) = R;
	h(0,2) = Ab(0,2);
	h(1,2) = Ab(1,2);
	frameTransform = PoseSE2(h);

	Eigen::Matrix<double,4,4> H = Eigen::Matrix<double,4,4>::Identity();
	H.block<2,2>(1,1) = R;
	H(1,3) = -Ab(0,2) * _scale; // Image x corresponds to camera -y
	H(2,3) = -Ab(1,2) * _scale; // Image y corresponds to camera -z
	transform = PoseSE3(H);
	return true;
	
}
	
}
