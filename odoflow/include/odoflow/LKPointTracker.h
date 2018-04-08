#pragma once

#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "paraset/ParameterManager.hpp"

#include "odoflow/InterestPointTracker.h"

namespace argus
{
/*! \brief Finds correspondences in images using the Lucas-Kanade
 * optical flow algorithm.
 */
class LKPointTracker
	: public InterestPointTracker
{
public:

	typedef std::shared_ptr<LKPointTracker> Ptr;

	LKPointTracker( ros::NodeHandle& nh, ros::NodeHandle& ph );

	virtual bool TrackInterestPoints( const cv::Mat& key,
	                                  InterestPoints& keyPoints,
	                                  const cv::Mat& tar,
	                                  InterestPoints& tarPoints );

private:

	// Flow calculation parameters
	NumericParam _pyramidLevel;
	NumericParam _flowWindowDim;
	NumericParam _logFlowEigenThreshold;
	NumericParam _solverMaxIters;
	NumericParam _solverMinLogEpsilon;
	NumericParam _maxFlowError;

	cv::TermCriteria _flowTermCriteria;

	cv::Size _flowWindowSize;
};
} // end namespace argus
