#pragma once

#include "odoflow/InterestPointDetector.h"

#include <opencv2/features2d.hpp>

namespace argus
{

/*! \brief FAST interest point detector. Uses OpenCV implementation. 
 * Params in private namespace: [type] {range} (default)
 * detector:
 *   intensity_threshold: [int] {0,max} (20)
 *   enable_non_max_suppression: [bool] (true)
 *   detector_type: [string] {FAST_5_8, FAST_7_12, FAST_9_16} (FAST_9_16)
 */
class FASTPointDetector
: public InterestPointDetector
{
public:
	
	typedef std::shared_ptr<FASTPointDetector> Ptr;
	
	FASTPointDetector( ros::NodeHandle& nh, ros::NodeHandle& ph );
	
	virtual InterestPoints FindInterestPoints( const cv::Mat& image );
	
private:
	
	cv::Ptr< cv::FastFeatureDetector > detector;
	
};

} // end namespace argus
