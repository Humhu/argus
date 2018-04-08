#include "odoflow/LKPointTracker.h"
#include "argus_utils/utils/ParamUtils.h"

#include <iostream>
#include <sstream>

namespace argus
{
LKPointTracker::LKPointTracker( ros::NodeHandle& nh, ros::NodeHandle& ph )
	: InterestPointTracker( nh, ph )
{
	_solverMaxIters.InitializeAndRead( ph, 30, "max_iters",
	                                   "Lucas-Kanade solver max iterations." );
	_solverMaxIters.AddCheck<GreaterThan>( 0 );
	_solverMaxIters.AddCheck<IntegerValued>( ROUND_CEIL );

	_solverMinLogEpsilon.InitializeAndRead( ph, -3, "log_min_eps",
	                                        "Lucas-Kande solver log10 min epsilon." );

	_pyramidLevel.InitializeAndRead( ph, 0, "pyramid_level",
	                                 "Lucas-Kanade max pyramid level." );
	_pyramidLevel.AddCheck<GreaterThanOrEqual>( 0 );
	_pyramidLevel.AddCheck<IntegerValued>( ROUND_CLOSEST );

	// TODO Normalize to images size
	_flowWindowDim.InitializeAndRead( ph, 30, "window_dim",
	                                  "Lucas-Kanade search window dim." );
	_flowWindowDim.AddCheck<GreaterThanOrEqual>( 3 ); // OpenCV requirements
	_flowWindowDim.AddCheck<IntegerValued>( ROUND_CLOSEST );

	_logFlowEigenThreshold.InitializeAndRead( ph, -4, "log_flow_eigenvalue_threshold",
	                                          "Lucas-Kanade spatial gradient log10 eigenvalue threshold." );

	_maxFlowError.InitializeAndRead( ph, 10, "flow_error_threshold",
	                                 "Lucas-Kanade max solution error threshold." );
	_maxFlowError.AddCheck<GreaterThan>( 0 );
}

bool LKPointTracker::TrackInterestPoints( const cv::Mat& key,
                                          InterestPoints& keypoints,
                                          const cv::Mat& tar,
                                          InterestPoints& tarpoints )
{
	// Make sure we have points to track
	if( key.empty() || tar.empty() || keypoints.empty() )
	{
		return false;
	}

	// OpenCV's Lucas-Kanade requires single-precision floating point
	InterestPointsf keyPoints = DowncastInterestPoints( keypoints );
	InterestPointsf targetPoints = DowncastInterestPoints( tarpoints );
	if( targetPoints.empty() )
	{
		targetPoints = keyPoints;
	}

	cv::TermCriteria termCriteria = cv::TermCriteria( cv::TermCriteria::COUNT |
	                                                  cv::TermCriteria::EPS,
	                                                  _solverMaxIters,
	                                                  std::pow( 10, _solverMinLogEpsilon ) );
	std::vector<uchar> status;
	std::vector<float> errors;
	cv::calcOpticalFlowPyrLK( key,
	                          tar,
	                          keyPoints,
	                          targetPoints,
	                          status,
	                          errors,
	                          cv::Size( _flowWindowDim, _flowWindowDim ),
	                          _pyramidLevel,
	                          termCriteria,
	                          cv::OPTFLOW_USE_INITIAL_FLOW,
	                          std::pow( 10, _logFlowEigenThreshold ) );

	InterestPointsf keyInliers, targetInliers;
	for( unsigned int i = 0; i < status.size(); ++i )
	{
		if( status[i] == 1 && errors[i] < _maxFlowError )
		{
			keyInliers.push_back( keyPoints[i] );
			targetInliers.push_back( targetPoints[i] );
		}
	}

	keypoints = UpcastInterestPoints( keyInliers );
	tarpoints = UpcastInterestPoints( targetInliers );

	return !keyPoints.empty();
}
}
