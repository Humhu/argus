#pragma once

#include <ros/ros.h>

#include <opencv2/core/core.hpp>
#include "paraset/ParameterManager.hpp"

#include "argus_utils/geometry/PoseSE2.h"

namespace argus
{
/*! \brief Uses OpenCV's findTransformECC to match two images using
   direct intensity values.
 */
class ECCDenseTracker
{
public:

	ECCDenseTracker( ros::NodeHandle& nh, ros::NodeHandle& ph );

	/*! \brief Finds a transformation that matches from to to.
	   Returns success.
	 */
	bool TrackImages( const cv::Mat& to, const cv::Mat& from,
	                  PoseSE2& pose );

private:

	NumericParam _logMinEps;
	NumericParam _maxIters;
	NumericParam _logMinCorrelation;
};
}
