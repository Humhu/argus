#pragma once

#include "odoflow/OdoflowCommon.h"
#include "odoflow/InterestPointDetector.h"

namespace argus
{
class InterestPointTracker
{
public:

	typedef std::shared_ptr<InterestPointTracker> Ptr;

	InterestPointTracker( ros::NodeHandle& nh, ros::NodeHandle& ph )
		: nodeHandle( nh ), privHandle( ph ) {}

	virtual ~InterestPointTracker() {}

	/*! \brief Tracks points in first image to second image. If guess is empty,
	 * firstPoints are used as initialization. */
	virtual bool TrackInterestPoints( const cv::Mat& key,
	                                  InterestPoints& keyPoints,
	                                  const cv::Mat& tar,
	                                  InterestPoints& tarPoints ) = 0;
protected:

	ros::NodeHandle nodeHandle;
	ros::NodeHandle privHandle;
};
} // end namespace argus
