#include "odoflow/RigidEstimator.h"
#include "odoflow/OpenCVMod.h"

#include "argus_utils/utils/ParamUtils.h"

#include <Eigen/SVD>

namespace argus
{

RigidEstimator::RigidEstimator( ros::NodeHandle& nh, ros::NodeHandle& ph )
: MotionEstimator( nh, ph )
{
	GetParam<double>( ph, "scale", _scale );

	double initReprojThreshold;
	GetParam<double>( ph, "reprojection_threshold", initReprojThreshold);
	_reprojThreshold.Initialize( ph, initReprojThreshold, "reprojection_threshold", 
	                             "RANSAC reprojection inlier threshold" );

	unsigned int initMaxIters;
	GetParam<unsigned int>( ph, "max_iters", initMaxIters );
	_maxIters.Initialize( ph, initMaxIters, "max_iters",
	                      "RANSAC max iterations" );
}

bool RigidEstimator::EstimateMotion( const InterestPoints& srcPoints,
                                     const InterestPoints& dstPoints,
                                     std::vector<uchar>& inliers,
                                     PoseSE3& transform,
                                     PoseSE2& frameTransform )
{
       

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
