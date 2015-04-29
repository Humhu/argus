#ifndef _ODOFLOW_POINT_TRACKER_H_
#define _ODOFLOW_POINT_TRACKER_H_

#include "odoflow/InterestPointDetector.h"

namespace odoflow
{
	
	class InterestPointTracker
	{
	public:
		
		typedef std::shared_ptr<InterestPointTracker> Ptr;
		
		InterestPointTracker();
		
		/*! \brief Tracks points in first image to second image. If guess is empty,
		 * firstPoints are used as initialization. */
		virtual void TrackInterestPoints( const cv::Mat& firstImage,
										  const cv::Mat& secondImage,
										  const InterestPoints& firstPoints,
										  const InterestPoints& secondPointsGuess,
										  std::vector<bool>& mask,
										  InterestPoints& firstInliers,
										  InterestPoints& secondInliers ) = 0;
	};
}

#endif
