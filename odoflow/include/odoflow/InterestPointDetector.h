#pragma once

#include <ros/ros.h>

#include <memory>
#include <vector>

#include <opencv2/video/tracking.hpp>

#include "odoflow/OdoflowCommon.h"

namespace argus
{
/*! \brief Base for classes that find interest points in an image for
 * optical tracking.
 */
class InterestPointDetector
{
public:

	typedef std::shared_ptr<InterestPointDetector> Ptr;

	InterestPointDetector( ros::NodeHandle& nh, ros::NodeHandle& ph )
		: nodeHandle( nh ), privHandle( ph ) {}

	virtual ~InterestPointDetector() {}

	/*! \brief Return interest points in a target image.
	 */
	virtual InterestPoints FindInterestPoints( const cv::Mat& image ) = 0;

protected:

	ros::NodeHandle nodeHandle;
	ros::NodeHandle privHandle;
};
} // end namespace argus
