#pragma once

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include "camplex/CameraCalibration.h"

#include "odoflow/InterestPointDetector.h"
#include "odoflow/InterestPointTracker.h"
#include "odoflow/MotionEstimator.h"

#include "argus_utils/PoseSE3.h"

namespace odoflow
{
	
/*! \brief A complete VO pipeline that consumes images and outputs velocity estimates.
 * Subscribes to "/image_raw" for the image source.
 * Publishes geometry_msgs/TwistWithCovarianceStamped on "/velocity_raw".
 * Private namespace parameters are:
 * redetection_threshold: [todo]
 * camera_pose: [PoseSE3] (0)
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
	bool ProcessImage( const cv::Mat& image, camplex::CameraCalibration calibration, 
					   argus_utils::PoseSE3& displacement );
	
	void SetDetector( InterestPointDetector::Ptr det );
	void SetTracker( InterestPointTracker::Ptr tr );
	void SetEstimator( MotionEstimator::Ptr es );
	
	void SetRedetectionThreshold( unsigned int t );
	void SetVisualization( bool enable );
	
private:
	
	ros::NodeHandle nodeHandle;
	ros::NodeHandle privHandle;
	
	ros::Publisher velPub;
	
	image_transport::ImageTransport imagePort;
	image_transport::CameraSubscriber imageSub;
	
	argus_utils::PoseSE3 cameraPose;
	ros::Time lastTime;
	
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
	
	void ImageCallback( const sensor_msgs::ImageConstPtr& msg,	
						const sensor_msgs::CameraInfoConstPtr& info_msg );
	
};

} // end namespace odoflow
