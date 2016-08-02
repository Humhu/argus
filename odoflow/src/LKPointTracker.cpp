#include "odoflow/LKPointTracker.h"
#include "argus_utils/utils/ParamUtils.h"

#include <iostream>
#include <sstream>

namespace argus
{
	
LKPointTracker::LKPointTracker( ros::NodeHandle& nh, ros::NodeHandle& ph )
: InterestPointTracker( nh, ph )
{
	unsigned int initMaxIters;
	GetParamRequired<unsigned int>( ph, "max_iters", initMaxIters );
	_solverMaxIters.Initialize( ph, initMaxIters, "max_iters", 
	                            "Lucas-Kanade solver max iterations." );
	_solverMaxIters.AddCheck<GreaterThan>( 0 );

	double initMinEps;
	GetParamRequired( ph, "min_eps", initMinEps );
	_solverMinEpsilon.Initialize( ph, initMinEps, "min_eps", 
	                              "Lucas-Kande solver min epsilon." );
	_solverMinEpsilon.AddCheck<GreaterThanOrEqual>( 0 );
	
	unsigned int initPyramidLevel;
	GetParamRequired( ph, "pyramid_level", initPyramidLevel );
	_pyramidLevel.Initialize( ph, initPyramidLevel, "pyramid_levels", 
	                          "Lucas-Kanade max pyramid level." );
	_pyramidLevel.AddCheck<GreaterThan>( 0 );

	unsigned int initWindowDim;
	GetParamRequired( ph, "window_dim", initWindowDim );
	_flowWindowDim.Initialize( ph, initWindowDim, "window_dim", 
	                           "Lucas-Kanade search window dim." );
	_flowWindowDim.AddCheck<GreaterThanOrEqual>( 0 );

	double initFlowThreshold;
	GetParamRequired( ph, "flow_eigenvalue_threshold", initFlowThreshold );
	_flowEigenThreshold.Initialize( ph, initFlowThreshold, "flow_eigenvalue_threshold", 
	                                "Lucas-Kanade spatial gradient eigenvalue threshold." );
	_flowEigenThreshold.AddCheck<GreaterThanOrEqual>( 0 );
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
	
	cv::TermCriteria termCriteria = cv::TermCriteria( cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 
	                                                 _solverMaxIters, _solverMinEpsilon );
	std::vector<uchar> status;
	std::vector<float> errors;
	cv::Size winSize( _flowWindowDim, _flowWindowDim );
	cv::calcOpticalFlowPyrLK( firstImage, 
	                          secondImage, 
	                          firstConvertedPoints, 
	                          secondConvertedPoints, 
	                          status, 
	                          errors, 
	                          winSize,
	                          _pyramidLevel,
	                          termCriteria, 
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
