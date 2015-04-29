#ifndef _OFLOW_VO_PIPELINE_H_
#define _OFLOW_VO_PIPELINE_H_

#include "odoflow/InterfaceBindings.h"

#include "odoflow/InterestPointDetector.h"
#include "odoflow/InterestPointTracker.h"
#include "odoflow/MotionEstimator.h"

#include "odoflow/PoseSE3.h"

namespace odoflow
{
	
	class VisualOdometryPipeline
	{
	public:
		
		typedef std::shared_ptr<VisualOdometryPipeline> Ptr;
		
		VisualOdometryPipeline();
		~VisualOdometryPipeline();
		
		/*! \brief Returns success. */
		bool ProcessImage( const cv::Mat& image, const Timepoint& timestamp,
						   PoseSE3& displacement );
		
		void SetDetector( InterestPointDetector::Ptr det );
		void SetTracker( InterestPointTracker::Ptr tr );
		void SetEstimator( MotionEstimator::Ptr es );
		
		void SetRedetectionThreshold( unsigned int t );
		void SetVisualization( bool enable );
		
		void SetRectificationParameters( const cv::Matx33d& cameraMat,
										 const cv::Mat& distortionCoeffs );
		
	private:
		
		InterestPointDetector::Ptr detector;
		InterestPointTracker::Ptr tracker;
		MotionEstimator::Ptr estimator;
		bool showOutput;
		
		cv::Mat keyframe;
		Timepoint keyframeTime;
		InterestPoints keyframePoints;
		
		cv::Mat midframe;
		InterestPoints midframePoints;
		
		unsigned int redetectionThreshold;
		
		cv::Matx33d cameraMatrix;
		cv::Mat distortionCoefficients;
		
	};
}

#endif
