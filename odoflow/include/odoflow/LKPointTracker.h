#pragma once

#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "odoflow/InterestPointTracker.h"

namespace odoflow
{
	
class LKPointTracker
	: public InterestPointTracker
{
public:
	
	typedef std::shared_ptr<LKPointTracker> Ptr;
	
	LKPointTracker( ros::NodeHandle& nh, ros::NodeHandle& ph );
	
	virtual void TrackInterestPoints( const cv::Mat& firstImage,
	                                  const cv::Mat& secondImage,
	                                  const InterestPoints& firstPoints,
	                                  const InterestPoints& secondPointsGuess,
	                                  InterestPoints& firstInliers,
	                                  InterestPoints& secondInliers );
	
	void SetFlowCriteria( int maxIters, double epsilon );
	void SetFlowWindow( int width, int height );
	void SetFlowThreshold( double eig );
	
private:
	
	// Flow calculation parameters
	cv::TermCriteria flowTermCriteria;
	cv::Size flowWindowSize;
	double flowEigenThreshold;
	
};

} // end namespace odoflow
