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
	
	virtual bool TrackInterestPoints( FrameInterestPoints& key,
	                                  FrameInterestPoints& tar );
	
	void SetFlowCriteria( int maxIters, double epsilon );
	void SetFlowWindow( int width, int height );
	void SetFlowThreshold( double eig );
	
private:
	
	// Flow calculation parameters
	NumericParam _pyramidLevel;
	NumericParam _flowWindowDim;
	NumericParam _flowEigenThreshold;
	NumericParam _solverMaxIters;
	NumericParam _solverMinLogEpsilon;
	NumericParam _maxFlowError;

	cv::TermCriteria _flowTermCriteria;

	cv::Size _flowWindowSize;

};

} // end namespace argus
