#pragma once

#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "paraset/ParameterManager.hpp"

#include "odoflow/InterestPointTracker.h"

namespace argus
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
	// int pyramidLevels;
	ParameterManager<unsigned int> _pyramidLevels;

	cv::TermCriteria _flowTermCriteria;

	// cv::Size _flowWindowSize;
	ParameterManager<cv::Size> _flowWindowSize;

	double _flowEigenThreshold;
	
};

} // end namespace argus
