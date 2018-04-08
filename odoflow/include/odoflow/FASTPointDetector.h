#pragma once

#include "odoflow/InterestPointDetector.h"
#include "paraset/ParameterManager.hpp"

namespace argus
{
/*! \brief FAST interest point detector. Uses OpenCV implementation.
 */
class FASTPointDetector
	: public InterestPointDetector
{
public:

	typedef std::shared_ptr<FASTPointDetector> Ptr;

	FASTPointDetector( ros::NodeHandle& nh, ros::NodeHandle& ph );

	virtual InterestPoints FindInterestPoints( const cv::Mat& image );

private:

	NumericParam _intensityThreshold;
	BooleanParam _enableNMS;
	NumericParam _maxNumPoints;
	StringParam _detectorType;

	static int StringToDetector( const std::string& str );
};
} // end namespace argus
