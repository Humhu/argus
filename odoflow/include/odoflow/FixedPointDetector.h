#ifndef _OFLOW_FIXEDFINDER_H_
#define _OFLOW_FIXEDFINDER_H_

#include "odoflow/InterestPointDetector.h"

namespace odoflow
{

	/*! \class FixedPointDetector FixedPointDetector.h
	 *	\brief Returns a constant set of points in an image for 
	 * optical flow tracking. 
	 */
	class FixedPointDetector 
	: public InterestPointDetector
	{
	public:
	
		typedef std::shared_ptr<FixedPointDetector> Ptr;
		
		FixedPointDetector();
		
		/*! \brief Return fixed interest points. 
		 * Image is used only for size scaling. */
		virtual InterestPoints FindInterestPoints( const cv::Mat& image );
		
		/*! \brief Set the points returned by this finder. Points
		 * should be specified [x,y] with each element between [0,1]
		 * representing relative coordinates in the image. */
		void SetPoints( const InterestPoints& pts );
		
	private:
		
		InterestPoints points;
	};
	
}


#endif
