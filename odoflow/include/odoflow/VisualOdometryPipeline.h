#pragma once

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>

#include "odoflow/InterestPointDetector.h"
#include "odoflow/InterestPointTracker.h"
#include "odoflow/MotionEstimator.h"

#include "argus_utils/geometry/PoseSE3.h"
#include "argus_utils/geometry/PoseSE2.h"

#include "lookup/LookupInterface.h"
#include "extrinsics_array/ExtrinsicsInfoManager.h"

#include "paraset/ParameterManager.hpp"

#include "broadcast/BroadcastTransmitter.h"

#include <unordered_set>

namespace argus
{

// TODO Output displacement_raw directly from tracking estimates
// TODO Abstract keyframe, redetection policies?
// TODO Covariance estimator interface

/*! \brief A complete VO pipeline that consumes images and outputs velocity estimates.
 * Subscribes to "/image_raw" for the image source. Uses the lookup system to
 * get extrinsics for cameras.
 *
 * detector:
 *   type: [string] {corner, fixed, FAST} (corner)
 * tracker:
 *   type: [string] {lucas_kanade} (lucas_kanade)
 * estimator:
 *   type: [string] {rigid} (rigid) */
class VisualOdometryPipeline
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	typedef std::shared_ptr<VisualOdometryPipeline> Ptr;
	
	VisualOdometryPipeline( ros::NodeHandle& nh, ros::NodeHandle& ph );
	~VisualOdometryPipeline();
	
	void SetDetector( InterestPointDetector::Ptr det );
	void SetTracker( InterestPointTracker::Ptr tr );
	void SetEstimator( MotionEstimator::Ptr es );
	
	void SetRedetectionThreshold( unsigned int t );
	void SetVisualization( bool enable );
	
private:
	
	ros::NodeHandle _nodeHandle;
	ros::NodeHandle _privHandle;
	
	image_transport::ImageTransport _imagePort;

	BroadcastTransmitter _featureTx;
	
	LookupInterface _lookupInterface;
	ExtrinsicsInfoManager _extrinsicsManager;
	
	InterestPointDetector::Ptr _detector;
	InterestPointTracker::Ptr _tracker;
	MotionEstimator::Ptr _estimator;
	
	bool _showOutput;
	
	struct CameraRegistration
	{
		Mutex mutex;
		
		unsigned int framesSkipped;
		std::string name;
		bool showOutput;
		image_transport::CameraSubscriber imageSub;
		image_transport::Publisher debugPub;
		ros::Publisher velPub;

		FrameInterestPoints keyFrame;
		FrameInterestPoints lastFrame;

		size_t originalNumKeypoints; // Number of keypoints on detection
		PoseSE3 lastPointsPose;

	};
	std::unordered_map<std::string, CameraRegistration> _cameraRegistry;
	
	// Need this to avoid double-subscribing
	std::unordered_set<std::string> _imageTopics;

	NumericParam _minNumKeypoints;
	NumericParam _redetectionThreshold;
	NumericParam _minInlierRatio;
	NumericParam _subsampleRate;
	NumericParam _maxFrameDt;

	PoseSE3::CovarianceMatrix _obsCovariance;

	void RegisterCamera( const std::string& name, const YAML::Node& info );

	void VisualizeFrame( const CameraRegistration& registration );
	
	void ImageCallback( const sensor_msgs::ImageConstPtr& msg,
						const sensor_msgs::CameraInfoConstPtr& info_msg );
	void SetKeyframe( CameraRegistration& registration,
	                  const FrameInterestPoints& key );
	
};

} // end namespace odoflow
