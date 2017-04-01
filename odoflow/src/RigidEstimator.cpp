#include "odoflow/RigidEstimator.h"
#include "argus_utils/utils/ParamUtils.h"

#include <opencv2/calib3d.hpp>
#include <Eigen/SVD>

namespace argus
{

RigidEstimator::RigidEstimator( ros::NodeHandle& nh, ros::NodeHandle& ph )
: MotionEstimator( nh, ph )
{
	GetParam( ph, "scale", _scale );

	double logReprojThreshold;
	GetParam( ph, "log_reprojection_threshold", logReprojThreshold);
	_logReprojThreshold.Initialize( ph, logReprojThreshold, 
	                                "log_reprojection_threshold", 
	                                "RANSAC reprojection inlier threshold" );
	_logReprojThreshold.AddCheck<GreaterThan>( 0 );
	_logReprojThreshold.AddCheck<LessThan>( 1.0 );

	unsigned int maxIters;
	GetParam( ph, "max_iters", maxIters );
	_maxIters.Initialize( ph, maxIters, "max_iters",
	                      "RANSAC max iterations" );
	_maxIters.AddCheck<GreaterThan>( 0 );
	_maxIters.AddCheck<IntegerValued>( ROUND_CEIL );
}

bool RigidEstimator::EstimateMotion( FrameInterestPoints& key,
                                     FrameInterestPoints& tar,
                                     PoseSE3& transform )
{
	if( key.points.empty() || tar.points.empty() )
	{
		ROS_INFO_STREAM( "RigidEstimator: Received empty points." );
		return false;
	}


	std::vector<char> inliers;
	FrameInterestPoints keyNormalized = key.Normalize();
	FrameInterestPoints tarNormalized = tar.Normalize();

	// We want tar in frame of key, so this is the ordering
	cv::Mat Hxest = cv::findHomography( tarNormalized.points, 
	                                    keyNormalized.points, 
	                                    cv::RANSAC, 
	                                    exp10(_logReprojThreshold),
	                                    inliers, 
	                                    _maxIters );
	if( Hxest.empty() )
	{
		ROS_INFO_STREAM( "RigidEstimator: Failed to find homography." );
		return false;
	}

	InterestPoints keyInliers, tarInliers;
	for( unsigned int i = 0; i < inliers.size(); i++ )
	{
		if( inliers[i] )
		{
			keyInliers.push_back( key.points[i] );
			tarInliers.push_back( tar.points[i] );
		}
	}
	key.points = keyInliers;
	tar.points = tarInliers;

	// cv::Mat Hest = cv::estimateRigidTransform( key, tar, false );
	// if( Hest.size().height == 0 || Hest.size().width == 0 )
	// {
	// 	ROS_WARN_STREAM( "RigidEstimator: Could not estimate motion." );
	// 	return false;
	// }

	Eigen::MatrixXd Ab = MatToEigen<double>( Hxest );

	// Extract the rotation using Procrustes solution
	Eigen::Matrix2d A = Ab.block<2,2>(0,0);
	Eigen::JacobiSVD<Eigen::Matrix2d> svd( A, Eigen::ComputeFullU | Eigen::ComputeFullV );
	Eigen::Matrix2d R = svd.matrixU() * svd.matrixV().transpose();
	
	// ROS_INFO_STREAM( "Ab: " << std::endl << Ab );
	// ROS_INFO_STREAM( "R: " << std::endl << R );

	FixedMatrixType<4,4> H = FixedMatrixType<4,4>::Identity();
	H.block<2,2>(1,1) = R;
	H(1,3) = -Ab(0,2) * _scale; // Image x corresponds to camera -y
	H(2,3) = -Ab(1,2) * _scale; // Image y corresponds to camera -z
	transform = PoseSE3(H);
	return true;
	
}

}
