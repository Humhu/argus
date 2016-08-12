#include "odoflow/LKPointTracker.h"
#include "argus_utils/utils/ParamUtils.h"

#include <iostream>
#include <sstream>

namespace argus
{
	
LKPointTracker::LKPointTracker( ros::NodeHandle& nh, ros::NodeHandle& ph )
: InterestPointTracker( nh, ph )
{
	FullNumericRange maxIters;
	GetParamRequired( ph, "max_iters", maxIters );
	_solverMaxIters.Initialize( ph, maxIters.init, "max_iters", 
	                            "Lucas-Kanade solver max iterations." );
	_solverMaxIters.AddCheck<GreaterThanOrEqual>( maxIters.min );
	_solverMaxIters.AddCheck<LessThanOrEqual>( maxIters.max );
	_solverMaxIters.AddCheck<IntegerValued>( ROUND_CEIL );

	MaxNumericRange minEps;
	GetParamRequired( ph, "min_eps", minEps );
	_solverMinEpsilon.Initialize( ph, minEps.init, "min_eps", 
	                              "Lucas-Kande solver min epsilon." );
	_solverMinEpsilon.AddCheck<GreaterThanOrEqual>( 0 );
	_solverMinEpsilon.AddCheck<LessThanOrEqual>( minEps.max );
	
	MaxNumericRange pyramidLevel;
	GetParamRequired( ph, "pyramid_level", pyramidLevel );
	_pyramidLevel.Initialize( ph, pyramidLevel.init, "pyramid_level", 
	                          "Lucas-Kanade max pyramid level." );
	_pyramidLevel.AddCheck<GreaterThanOrEqual>( 0 );
	_pyramidLevel.AddCheck<LessThanOrEqual>( pyramidLevel.max );
	_pyramidLevel.AddCheck<IntegerValued>( ROUND_CLOSEST );

	MaxNumericRange windowDim;
	GetParamRequired( ph, "window_dim", windowDim );
	_flowWindowDim.Initialize( ph, windowDim.init, "window_dim", 
	                           "Lucas-Kanade search window dim." );
	_flowWindowDim.AddCheck<GreaterThanOrEqual>( 3 ); // OpenCV requirements
	_flowWindowDim.AddCheck<LessThanOrEqual>( windowDim.max );
	_flowWindowDim.AddCheck<IntegerValued>( ROUND_CLOSEST );

	MaxNumericRange flowThreshold;
	_flowEigenThreshold.Initialize( ph, flowThreshold.init, "flow_eigenvalue_threshold", 
	                                "Lucas-Kanade spatial gradient eigenvalue threshold." );
	_flowEigenThreshold.AddCheck<GreaterThanOrEqual>( 0 );
	_flowEigenThreshold.AddCheck<LessThanOrEqual>( flowThreshold.max );
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
