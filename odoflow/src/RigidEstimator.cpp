#include "odoflow/RigidEstimator.h"
#include "camplex/FiducialCommon.h"
#include "argus_utils/utils/ParamUtils.h"

#include <opencv2/calib3d.hpp>
#include <Eigen/SVD>

namespace argus
{
RigidEstimator::RigidEstimator( ros::NodeHandle& nh, ros::NodeHandle& ph )
{
	double logReprojThreshold;
	GetParam( ph, "log_reprojection_threshold", logReprojThreshold );
	_logReprojThreshold.Initialize( ph, logReprojThreshold,
	                                "log_reprojection_threshold",
	                                "RANSAC reprojection inlier threshold" );

	unsigned int maxIters;
	GetParam( ph, "max_iters", maxIters );
	_maxIters.Initialize( ph, maxIters, "max_iters",
	                      "RANSAC max iterations" );
	_maxIters.AddCheck<GreaterThan>( 0 );
	_maxIters.AddCheck<IntegerValued>( ROUND_CEIL );
}

bool RigidEstimator::EstimateMotion( InterestPoints& key,
                                     InterestPoints& tar,
									 std::vector<unsigned int>& inlierInds,
                                     PoseSE2& transform  )
{
	if( key.empty() || tar.empty() )
	{
		ROS_INFO_STREAM( "RigidEstimator: Received empty points." );
		return false;
	}

	std::vector<char> inliers;
	// We want tar in frame of key, so this is the ordering
	cv::Mat Hxest = cv::findHomography( tar,
	                                    key,
	                                    cv::RANSAC,
	                                    std::pow( 10, _logReprojThreshold ),
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
			inlierInds.push_back(i);
		}
	}

	Eigen::MatrixXd Ab = MatToEigen<double>( Hxest );

	// Extract the rotation using Procrustes solution
	Eigen::Matrix2d A = Ab.block<2, 2>( 0, 0 );
	Eigen::JacobiSVD<Eigen::Matrix2d> svd( A, Eigen::ComputeFullU | Eigen::ComputeFullV );
	Eigen::Matrix2d R = svd.matrixU() * svd.matrixV().transpose();

	FixedMatrixType<3, 3> H = FixedMatrixType<3, 3>::Identity();
	H.block<2, 2>( 1, 1 ) = R;
	H( 1, 2 ) = -Ab( 0, 2 ); // Image x corresponds to camera -y
	H( 2, 2 ) = -Ab( 1, 2 ); // Image y corresponds to camera -z
	transform = PoseSE2( H );
	return true;
}
}
