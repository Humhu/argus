#ifndef _OFLOW_TRACKER_H_
#define _OFLOW_TRACKER_H_

#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "odoflow/InterestPointDetector.h"
#include "odoflow/InterfaceBindings.h"

#include <memory>

namespace odoflow
{

	class FlowTracker
	{
	public:
		
		typedef std::shared_ptr<FlowTracker> Ptr;
		
		FlowTracker( InterestPointDetector::Ptr detector );
		
		/*! \brief Process a new timestamped image. If no time is given, the current
			time is assumed as the timestamp. */
		void ProcessImage( const cv::Mat& image, 
						   const Timepoint& timestamp = Interface::GetTime() );
		
		void ToggleOutput( bool enable );
		
		void SetFlowCriteria( int maxIters, double epsilon );
		void SetFlowWindow( int width, int height );
		void SetFlowThreshold( double eig );
		
	private:
		
		bool showOutput;
		
		InterestPointDetector::Ptr interestPointDetector;
		
		// Flow calculation parameters
		cv::TermCriteria flowTermCrit;
		cv::Size flowWindowSize;
		double flowEigenThreshold;
		
		// State variables
		cv::Mat prevImage;
		Timepoint prevTime;
		
		std::vector<cv::Point2f> currPoints;
		
	};
	
}

#endif
