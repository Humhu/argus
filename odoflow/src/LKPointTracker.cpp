#include "odoflow/LKPointTracker.h"
#include "argus_utils/utils/ParamUtils.h"

#include <iostream>
#include <sstream>

namespace argus
{
	
LKPointTracker::LKPointTracker( ros::NodeHandle& nh, ros::NodeHandle& ph )
: InterestPointTracker( nh, ph ),
_pyramidLevels( ph, "pyramid_levels" ),
_flowWindowSize( ph, "window_size" )
{
	int maxIters;
	double minEps;
	GetParamRequired( ph, "max_iters", maxIters );
	GetParamRequired( ph, "min_eps", minEps );
	_flowTermCriteria = cv::TermCriteria( cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 
	                                     maxIters, minEps );
	
	std::vector<unsigned int> pyramidLevels;
	GetParamRequired( ph, "pyramid_levels", pyramidLevels );
	for( unsigned int i = 0; i < pyramidLevels.size(); i++ )
	{
		_pyramidLevels.AddSetting( pyramidLevels[i], std::to_string( pyramidLevels[i] ) );
	}
	
	std::vector<unsigned int> windowSizes;
	GetParamRequired( ph, "window_sizes", windowSizes );
	for( unsigned int i = 0; i < windowSizes.size(); i++ )
	{
		std::stringstream ss;
		ss << "[" << windowSizes[i] << ", " << windowSizes[i] << "]";
		_flowWindowSize.AddSetting( cv::Size( windowSizes[i], windowSizes[i] ),
		                            ss.str() );
	}
	
	GetParamRequired( ph, "eigenvalue_threshold", _flowEigenThreshold );
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
	cv::calcOpticalFlowPyrLK( firstImage, 
	                          secondImage, 
	                          firstConvertedPoints, 
	                          secondConvertedPoints, 
	                          status, 
	                          errors, 
	                          _flowWindowSize.GetValue(), 
	                          _pyramidLevels.GetValue(), 
	                          _flowTermCriteria, 
	                          cv::OPTFLOW_USE_INITIAL_FLOW, 
	                          _flowEigenThreshold ); 
	
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
