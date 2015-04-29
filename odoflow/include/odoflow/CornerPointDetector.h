#ifndef _OFLOW_CORNERFINDER_H_
#define _OFLOW_CORNERFINDER_H_

#include "odoflow/InterestPointDetector.h"

// TODO This class is not thread-safe!
namespace odoflow
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
		
		CornerPointDetector();
		
		/*! \brief Return interest points in a target grayscale image. */
		virtual InterestPoints FindInterestPoints( const cv::Mat& image );
		
		/*! \brief The feature finding setters. */
		void SetMaxPoints( int maxPoints );
		void SetMinQuality( double quality );
		void SetMinSeparation( double separation );
		void SetQualityWindowSize( int winSize );
		void SetHarrisMode( bool enable );
		void SetHarrisK( double k );
		
		/*! \brief The corner subpixel refinement setters. */
		void SetRefinementMode( bool enable );
		void SetRefinementWindow( int width, int height );
		void SetRefinementCriteria( int maxIters, double epsilon );
		
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
	
}


#endif
