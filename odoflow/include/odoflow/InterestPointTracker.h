#pragma once

#include "odoflow/InterestPointDetector.h"

namespace argus
{
	
class InterestPointTracker
{
public:
	
	typedef std::shared_ptr<InterestPointTracker> Ptr;
	
	InterestPointTracker( ros::NodeHandle& nh, ros::NodeHandle& ph )
	: nodeHandle( nh ), privHandle( ph ) {}
	
	/*! \brief Tracks points in first image to second image. If guess is empty,
		* firstPoints are used as initialization. */
	virtual void TrackInterestPoints( const cv::Mat& firstImage,
	                                  const cv::Mat& secondImage,
	                                  const InterestPoints& firstPoints,
	                                  const InterestPoints& secondPointsGuess,
	                                  InterestPoints& firstInliers,
                                      InterestPoints& secondInliers ) = 0;
protected:
	
	ros::NodeHandle nodeHandle;
	ros::NodeHandle privHandle;
};

} // end namespace argus
