#include "odoflow/LKPointTracker.h"

#include <iostream>

namespace odoflow
{
	
	LKPointTracker::LKPointTracker( ros::NodeHandle& nh, ros::NodeHandle& ph )
	: InterestPointTracker( nh, ph )
	{
		int maxIters;
		double minEps;
		privHandle.param( "tracker/max_iters", maxIters, 20 );
		privHandle.param<double>( "tracker/min_eps", minEps, 0.03 );
		flowTermCriteria = cv::TermCriteria( cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 
											 maxIters, minEps );
		
		int windowDim;
		privHandle.param( "tracker/window_size", windowDim, 13 );
		flowWindowSize = cv::Size( windowDim, windowDim );
		// TODO Arg checking
		
		privHandle.param<double>( "tracker/eigenvalue_threshold", flowEigenThreshold, 0.001 );
	}
	
	void LKPointTracker::TrackInterestPoints( const cv::Mat& firstImage,
											  const cv::Mat& secondImage,
											  const InterestPoints& firstPoints,
											  const InterestPoints& secondPointsGuess,
											  InterestPoints& firstInliers,
											  InterestPoints& secondInliers )
	{
		firstInliers.clear();
		secondInliers.clear();
		
		// Make sure we have points to track
		if( firstImage.empty() || secondImage.empty() || firstPoints.empty() )
		{
			return;
		}
		
		// OpenCV's Lucas-Kanade requires single-precision floating point
		std::vector<cv::Point2f> firstConvertedPoints( firstPoints.size() );
		std::vector<cv::Point2f> secondConvertedPoints( secondPointsGuess.size() );
		for( unsigned int i = 0; i < firstPoints.size(); i++ )
		{
			firstConvertedPoints[i] = cv::Point2f( firstPoints[i].x, firstPoints[i].y );
		}
		
		if( secondPointsGuess.size() == firstPoints.size() )
		{
			for( unsigned int i = 0; i < secondPointsGuess.size(); i++ )
			{
				secondConvertedPoints[i] = cv::Point2f( secondPointsGuess[i].x, secondPointsGuess[i].y );
			}
		}
		else
		{
			secondConvertedPoints = firstConvertedPoints;
		}
		
		std::vector<uchar> status;
		std::vector<float> errors;
		cv::calcOpticalFlowPyrLK( firstImage, secondImage, firstConvertedPoints, secondConvertedPoints, 
								  status, errors, flowWindowSize, 3, flowTermCriteria, 
								  cv::OPTFLOW_USE_INITIAL_FLOW, flowEigenThreshold ); 
		
		// 2. Grab good matches
		firstInliers.reserve( firstPoints.size() );
		secondInliers.reserve( firstPoints.size() );
		for( unsigned int i = 0; i < firstPoints.size(); i++)
		{
			if( !status[i] ) { 
				continue; 
			}
			firstInliers.push_back( InterestPoint( firstConvertedPoints[i].x, firstConvertedPoints[i].y ) );
			secondInliers.push_back( InterestPoint( secondConvertedPoints[i].x, secondConvertedPoints[i].y ) );
		}
	}
	
}
