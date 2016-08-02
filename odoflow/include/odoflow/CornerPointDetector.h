#pragma once

#include "odoflow/InterestPointDetector.h"
#include "paraset/ParameterManager.hpp"

// TODO This class is not thread-safe!
namespace argus
{

/*! \class CornerPointDetector CornerPointDetector.h
	*	\brief Finds strong corner interest points in an image for 
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
	
	IntegerParameter _featureMaxPoints;	
	FloatParameter _featureMinQuality;
	FloatParameter _featureMinSeparation;
	IntegerParameter _featureBlockDim;
	
	BoolParameter _useHarris;
	FloatParameter _harrisK;

	BoolParameter _enableRefine;
	IntegerParameter _refineWindowDim;
	IntegerParameter _refineZeroDim;
	IntegerParameter _refineMaxIters;
	IntegerParameter _refineMinEps;
	
};
	
} // end namespace

