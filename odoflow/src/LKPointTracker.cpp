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
	GetParamRequired( ph, "flow_eigenvalue_threshold", flowThreshold );
	_flowEigenThreshold.Initialize( ph, flowThreshold.init, "flow_eigenvalue_threshold", 
	                                "Lucas-Kanade spatial gradient eigenvalue threshold." );
	_flowEigenThreshold.AddCheck<GreaterThanOrEqual>( 0 );
	_flowEigenThreshold.AddCheck<LessThanOrEqual>( flowThreshold.max );

	double flowErrThreshold;
	GetParamRequired( ph, "flow_error_threshold", flowErrThreshold );
	_maxFlowError.Initialize( ph, flowErrThreshold, "flow_error_threshold", 
	                                "Lucas-Kanade max solution error threshold." );
	_maxFlowError.AddCheck<GreaterThanOrEqual>( 0 );
}

bool LKPointTracker::TrackInterestPoints( FrameInterestPoints& key,
	                                      FrameInterestPoints& tar )
{
	// Make sure we have points to track
	if( key.frame.empty() || tar.frame.empty() || key.points.empty() )
	{
		return false;
	}
	
	// OpenCV's Lucas-Kanade requires single-precision floating point
	InterestPointsf keyPoints = DowncastInterestPoints( key.points );
	InterestPointsf targetPoints = DowncastInterestPoints( tar.points );
	if( targetPoints.empty() )
	{
		targetPoints = keyPoints;
	}
	
	cv::TermCriteria termCriteria = cv::TermCriteria( cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 
	                                                  _solverMaxIters, 
	                                                  _solverMinEpsilon );
	std::vector<uchar> status;
	std::vector<float> errors;
	cv::calcOpticalFlowPyrLK( key.frame, 
	                          tar.frame, 
	                          keyPoints, 
	                          targetPoints, 
	                          status, 
	                          errors, 
	                          cv::Size( _flowWindowDim, _flowWindowDim ),
	                          _pyramidLevel,
	                          termCriteria, 
	                          cv::OPTFLOW_USE_INITIAL_FLOW, 
	                          _flowEigenThreshold );
	
	InterestPointsf keyInliers, targetInliers;
	for( unsigned int i = 0; i < status.size(); ++i )
	{
		if( status[i] == 0 || errors[i] < _maxFlowError )
		{
			keyInliers.push_back( keyPoints[i] );
			targetInliers.push_back( targetPoints[i] );
		}
	}

	// 2. Grab good matches
	key.points = UpcastInterestPoints( keyInliers );
	tar.points = UpcastInterestPoints( targetInliers );

	return !key.points.empty();
}
	
}
