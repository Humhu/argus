#ifndef _OFLOW_POINTFINDER_H_
#define _OFLOW_POINTFINDER_H_

#include <memory>
#include <vector>

#include <opencv2/video/tracking.hpp>

namespace odoflow
{

	typedef cv::Point2d InterestPoint;
	typedef std::vector<InterestPoint> InterestPoints;
	
	/*! \class InterestPointDetector InterestPointDetector.h
	 *	\brief Base for classes that find interest points in an image for 
	 * optical flow tracking. 
	 */
	class InterestPointDetector
	{
	public:
	
		typedef std::shared_ptr<InterestPointDetector> Ptr;
		
		InterestPointDetector();
		
		/*! \brief Return interest points in a target image. Type of image
		 * required depends on point finder instantiation. */
		virtual InterestPoints FindInterestPoints( const cv::Mat& image ) = 0;
		
	};
	
}


#endif
