#pragma once

#include "odoflow/InterestPointDetector.h"

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
	
	/*! \brief The maximum number of corner points to return. */
	int featureMaxPoints;
	
	/*! \brief The minimum corner "quality" parameter. */
	double featureQuality;
	
	/*! \brief The minimum pixel distance between corners. */
	double featureSeparation;
	
	/*! \brief The window size over which to compute quality. */
	int featureBlockSize;
	
	/*! \brief Whether to use a Harris corner or eigenvalue detector. */
	bool featureUseHarris;
	
	/*! \brief The Harris K parameter. */
	double featureHarrisK;
	
	/*! \brief Enable sub-pixel refinement of corner candidates. */
	bool refineEnable;
	
	/*! \brief Window in which to perform refinement. */
	cv::Size refineWindowSize;
	
	/*! \brief Termination criteria for refinement. */
	cv::TermCriteria refineTermCriteria;
	
};
	
} // end namespace

