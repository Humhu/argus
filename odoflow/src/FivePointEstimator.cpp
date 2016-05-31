#include "odoflow/FivePointEstimator.h"
#include "odoflow/OpenCVMod.h"

#include "argus_utils/GeometryUtils.h"

#include <iostream>

using namespace argus;

namespace odoflow
{
	
	FivePointEstimator::FivePointEstimator( ros::NodeHandle& nh, ros::NodeHandle& ph )
	: MotionEstimator( nh, ph )
	{
		estimationMethod( RANSAC ), residualThreshold( 0.002 ), outputScale( 1.0 )	
		
		
	}
	
	bool FivePointEstimator::EstimateMotion( const InterestPoints& firstPoints,
											 const InterestPoints& secondPoints,
											 PoseSE3& transform )
	{
		
		transform = PoseSE3();
		
		// Need at least 5 points to use the 5-point algorithm!
		if( firstPoints.size() < 5 || secondPoints.size() < 5 )
		{
			std::cout << "Fewer than five points available. Cannot estimate motion." << std::endl;
			return false;
		}
		
		// 2. Estimate essential matrix using 5-point algorithm
		int iters = 0;
		double residual = std::numeric_limits<double>::infinity();
		cv::Mat R, t, mask, essentialCandidates, essential;
		
		essentialCandidates = 
			cv::findEssentialMat( firstPoints, secondPoints, 1.0,
								  cv::Point2d(0,0), ConvertMethod( estimationMethod ),
								  confidence, outlierThreshold, mask );
		
		// In some instances we will get multiple essential matrices back
		// For now, let's just take the first one
		essentialCandidates.rowRange(0,3).copyTo(essential);
		
		// 3. Find the correct rotation and translation by checking cheirality
		cv::Mat triangulatedPoints;
		int inliers = cv::RecoverPose( essential, firstPoints, secondPoints, R, t, 
									   triangulatedPoints, 1.0, cv::Point2d(0,0), mask,
// 									   10.0 );
									   std::numeric_limits<double>::infinity() );

		if( inliers < 5 )
		{
			return false;
		}
		
		double zSum = 0;
		for( unsigned int i = 0; i < triangulatedPoints.cols; i++ )
		{
			if( mask.at<uchar>(i) == 0 ) { continue; }
			cv::Mat euc = triangulatedPoints.col(i);
			zSum += euc.at<double>(2);
// 			std::cout << "Triangulated point to: " << VecToEigen<double,3>( euc ).transpose() << std::endl;;
		}
// 		zSum = zSum/inliers; // Average z height of inliers
// 		std::cout << "Average z of " << zSum << std::endl;
// 		double tScale = outputScale/zSum;
		double tScale = 0;
		
		Eigen::Matrix<double,4,4> H = Eigen::Matrix<double,4,4>::Identity();
		H.block<3,3>(0,0) = MatToEigen<double,3,3>( R );
		
		// TODO: The translation is always given as unit scale, this is incorrect!
		// Need to triangulate the points and use depth knowledge to disambiguate translation
		H.block<3,1>(0,3) = VecToEigen<double,3>( t )*tScale;

// 		std::cout << "Scale: " << tScale << std::endl;
// 		std::cout << "H:" << std::endl << H << std::endl;
		transform = PoseSE3( H );
		
		EulerAngles euler = QuaternionToEuler( transform.GetQuaternion() );
		std::cout << "Angle displacement: " << euler << std::endl;
		
		return true;
	}
	
	void FivePointEstimator::SetMethod( Method method )
	{
		estimationMethod = method;
	}
	
	void FivePointEstimator::SetFittingParameters( double prob, double thresh )
	{
		outlierThreshold = thresh;
		confidence = prob;
	}
	
	void FivePointEstimator::SetOutputScale( double scale )
	{
		outputScale = scale;
	}
	
	int FivePointEstimator::ConvertMethod( Method method )
	{
		switch( estimationMethod )
		{
			case RANSAC:
				return cv::RANSAC;
			case MEDS:
				return cv::LMEDS;
			default:
				throw std::runtime_error( "Invalid FivePoint Method" );
		}
	}
	
}
