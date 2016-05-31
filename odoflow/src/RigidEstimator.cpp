#include "odoflow/RigidEstimator.h"
#include "odoflow/OpenCVMod.h"
#include <Eigen/SVD>

using namespace argus;

namespace odoflow
{

RigidEstimator::RigidEstimator( ros::NodeHandle& nh, ros::NodeHandle& ph )
: MotionEstimator( nh, ph )
{
	ph.param<double>( "estimator/scale", scale, 1.0 );
	ph.param<double>( "estimator/reprojection_threshold", reprojThreshold, 3.0 );
}

bool RigidEstimator::EstimateMotion( const InterestPoints& srcPoints,
                                     const InterestPoints& dstPoints,
                                     std::vector<uchar>& inliers,
                                     PoseSE3& transform )
{
       

	cv::Mat Hxest = cv::findHomography( srcPoints, dstPoints, inliers, cv::RANSAC, reprojThreshold );

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
	
	ROS_INFO_STREAM( "Ab: " << std::endl << Ab );
	ROS_INFO_STREAM( "R: " << std::endl << R );
	
	Eigen::Matrix<double,4,4> H = Eigen::Matrix<double,4,4>::Identity();
	H.block<2,2>(1,1) = R;
	H(1,3) = -Ab(0,2) * scale; // Image x corresponds to camera -y
	H(2,3) = -Ab(1,2) * scale; // Image y corresponds to camera -z
	
	transform = PoseSE3(H);
	return true;
	
}
	
}
