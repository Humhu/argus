#include "odoflow/LKPointTracker.h"

#include <iostream>

namespace odoflow
{
	
	LKPointTracker::LKPointTracker()
	{}
	
	void LKPointTracker::TrackInterestPoints( const cv::Mat& firstImage,
											  const cv::Mat& secondImage,
											  const InterestPoints& firstPoints,
											  const InterestPoints& secondPointsGuess,
											  std::vector<bool>& mask,
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
		mask.reserve( firstPoints.size() );
		mask.clear();
		for( unsigned int i = 0; i < firstPoints.size(); i++)
		{
			if( !status[i] ) { 
				mask.push_back( false );
				continue; 
			}
			mask.push_back( true );
			firstInliers.push_back( InterestPoint( firstConvertedPoints[i].x, firstConvertedPoints[i].y ) );
			secondInliers.push_back( InterestPoint( secondConvertedPoints[i].x, secondConvertedPoints[i].y ) );
		}
	}
	
	void LKPointTracker::SetFlowCriteria( int maxIters, double epsilon )
	{
		flowTermCriteria = cv::TermCriteria( cv::TermCriteria::COUNT | cv::TermCriteria::EPS, maxIters, epsilon );
	}

	void LKPointTracker::SetFlowWindow( int width, int height )
	{
		flowWindowSize = cv::Size( width, height );
	}
	
	void LKPointTracker::SetFlowThreshold( double eig )
	{
		flowEigenThreshold = eig;
	}
	
}
