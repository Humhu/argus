#pragma once

#include "odoflow/InterestPointDetector.h"
#include "paraset/ParameterManager.hpp"

// TODO This class is not thread-safe!
namespace argus
{
/*! \brief Finds strong corner interest points in an image for
 * optical flow tracking.
 */
class CornerPointDetector
	: public InterestPointDetector
{
public:

	typedef std::shared_ptr<CornerPointDetector> Ptr;

	CornerPointDetector( ros::NodeHandle& nh, ros::NodeHandle& ph );

	/*! \brief Return interest points in a target grayscale image. */
	virtual InterestPoints FindInterestPoints( const cv::Mat& image );

private:

	NumericParam _featureMaxPoints;
	NumericParam _featureMinQuality;
	NumericParam _featureMinSeparation;
	NumericParam _featureBlockDim;

	BooleanParam _useHarris;
	NumericParam _harrisK;

	BooleanParam _enableRefine;
	NumericParam _refineWindowDim;
	NumericParam _refineZeroDim;
	NumericParam _refineMaxIters;
	NumericParam _refineMinLogEps;
};
} // end namespace
