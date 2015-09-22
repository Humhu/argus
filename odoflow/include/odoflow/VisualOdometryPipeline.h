#pragma once

#include "odoflow/InterestPointDetector.h"
#include "odoflow/InterestPointTracker.h"
#include "odoflow/MotionEstimator.h"

#include "argus_utils/PoseSE3.h"

namespace odoflow
{
	
/*! \brief A complete VO pipeline that consumes images and outputs velocity estimates. 
 * Requires four components in the private namespace:
 * detector:
 *   type: [string] {corner, fixed, FAST} (corner)
 * tracker:
 *   type: [string] {lucas_kanade} (lucas_kanade)
 * estimator:
 *   type: [string] {rigid} (rigid)
 */
// TODO Abstract keyframe, redetection policies?
class VisualOdometryPipeline
{
public:
	
	typedef std::shared_ptr<VisualOdometryPipeline> Ptr;
	
	VisualOdometryPipeline( ros::NodeHandle& nh, ros::NodeHandle& ph );
	~VisualOdometryPipeline();
	
	/*! \brief Returns success. */
	bool ProcessImage( const cv::Mat& image, argus_utils::PoseSE3& displacement );
	
	void SetDetector( InterestPointDetector::Ptr det );
	void SetTracker( InterestPointTracker::Ptr tr );
	void SetEstimator( MotionEstimator::Ptr es );
	
	void SetRedetectionThreshold( unsigned int t );
	void SetVisualization( bool enable );
	
	void SetRectificationParameters( const cv::Matx33d& cameraMat,
										const cv::Mat& distortionCoeffs );
	
private:
	
	ros::NodeHandle nodeHandle;
	ros::NodeHandle privHandle;
	
	InterestPointDetector::Ptr detector;
	InterestPointTracker::Ptr tracker;
	MotionEstimator::Ptr estimator;
	bool showOutput; // TODO Move into a separate node or something
	bool enableUndistortion;
	
	cv::Mat keyframe;
	InterestPoints keyframePoints;
	
	cv::Mat midframe;
	InterestPoints midframePoints;
	
	unsigned int redetectionThreshold;
	
	cv::Matx33d cameraMatrix;
	cv::Mat distortionCoefficients;
	
};

} // end namespace odoflow
