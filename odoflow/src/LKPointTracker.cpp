#include "odoflow/LKPointTracker.h"
#include "argus_utils/utils/ParamUtils.h"

#include <iostream>
#include <sstream>

namespace argus
{

LKPointTracker::LKPointTracker( ros::NodeHandle& nh, ros::NodeHandle& ph )
	: InterestPointTracker( nh, ph )
{
	unsigned int maxIters;
	GetParamRequired( ph, "max_iters", maxIters );
	_solverMaxIters.Initialize( ph, maxIters, "max_iters",
	                            "Lucas-Kanade solver max iterations." );
	_solverMaxIters.AddCheck<GreaterThan>( 0 );
	_solverMaxIters.AddCheck<IntegerValued>( ROUND_CEIL );

	double minEps;
	GetParamRequired( ph, "log_min_eps", minEps );
	_solverMinLogEpsilon.Initialize( ph, minEps, "log_min_eps",
	                                 "Lucas-Kande solver log min epsilon." );
	_solverMinLogEpsilon.AddCheck<GreaterThanOrEqual>( 0 );

	unsigned int pyramidLevel;
	GetParamRequired( ph, "pyramid_level", pyramidLevel );
	_pyramidLevel.Initialize( ph, pyramidLevel, "pyramid_level",
	                          "Lucas-Kanade max pyramid level." );
	_pyramidLevel.AddCheck<GreaterThanOrEqual>( 0 );
	_pyramidLevel.AddCheck<IntegerValued>( ROUND_CLOSEST );

	unsigned int windowDim;
	GetParamRequired( ph, "window_dim", windowDim );
	_flowWindowDim.Initialize( ph, windowDim, "window_dim",
	                           "Lucas-Kanade search window dim." );
	_flowWindowDim.AddCheck<GreaterThanOrEqual>( 3 ); // OpenCV requirements
	_flowWindowDim.AddCheck<IntegerValued>( ROUND_CLOSEST );

	double flowThreshold;
	GetParamRequired( ph, "flow_eigenvalue_threshold", flowThreshold );
	_flowEigenThreshold.Initialize( ph, flowThreshold, "flow_eigenvalue_threshold",
	                                "Lucas-Kanade spatial gradient eigenvalue threshold." );
	_flowEigenThreshold.AddCheck<GreaterThan>( 0 );

	double flowErrThreshold;
	GetParamRequired( ph, "flow_error_threshold", flowErrThreshold );
	_maxFlowError.Initialize( ph, flowErrThreshold, "flow_error_threshold",
	                          "Lucas-Kanade max solution error threshold." );
	_maxFlowError.AddCheck<GreaterThan>( 0 );
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

	cv::TermCriteria termCriteria = cv::TermCriteria( cv::TermCriteria::COUNT |
	                                                  cv::TermCriteria::EPS,
	                                                  _solverMaxIters,
	                                                  exp10( _solverMinLogEpsilon ) );
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
		if( status[i] == 1 && errors[i] < _maxFlowError )
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
