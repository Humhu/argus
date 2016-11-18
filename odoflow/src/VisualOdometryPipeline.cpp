#include "odoflow/VisualOdometryPipeline.h"

#include "odoflow/CornerPointDetector.h"
#include "odoflow/FixedPointDetector.h"
#include "odoflow/FASTPointDetector.h"

#include "odoflow/LKPointTracker.h"

#include "odoflow/RigidEstimator.h"

#include "argus_utils/geometry/GeometryUtils.h"
#include "argus_utils/utils/MatrixUtils.h"
#include "argus_utils/utils/ParamUtils.h"

#include "camplex/CameraCalibration.h"

#include <geometry_msgs/TwistStamped.h>

#include <opencv2/highgui/highgui.hpp>

namespace argus
{

VisualOdometryPipeline::VisualOdometryPipeline( ros::NodeHandle& nh, ros::NodeHandle& ph )
: _nodeHandle( nh ), 
_privHandle( ph ), 
_imagePort( _nodeHandle )
{
	double initRedectThresh;
	GetParamRequired( ph, "redetection_threshold", initRedectThresh );
	_redetectionThreshold.Initialize( ph, initRedectThresh, "redetection_threshold", 
	                                  "Pipeline feature redetection min threshold" );
	_redetectionThreshold.AddCheck<GreaterThanOrEqual>( 0 );
	_redetectionThreshold.AddCheck<LessThanOrEqual>( 1.0 );

	double initMinInlierRatio;
	GetParamRequired( ph, "min_inlier_ratio", initMinInlierRatio );
	_minInlierRatio.Initialize( ph, initMinInlierRatio, "min_inlier_ratio",
	                           "Pipeline min feature inlier threshold" );
	_minInlierRatio.AddCheck<GreaterThan>( 0 );
	_minInlierRatio.AddCheck<LessThanOrEqual>( 1.0 );

	double initSubsampleRate;
	GetParamRequired( ph, "subsample_rate", initSubsampleRate );
	_subsampleRate.Initialize( ph, initSubsampleRate, "subsample_rate",
	                           "Frame subsampling rate" );
	_subsampleRate.AddCheck<GreaterThanOrEqual>( 0 );
	_subsampleRate.AddCheck<IntegerValued>( ROUND_CLOSEST );

	double initMaxFrameDt;
	GetParamRequired( ph, "max_frame_dt", initMaxFrameDt );
	_maxFrameDt.Initialize( ph, initMaxFrameDt, "max_frame_dt",
	                        "Maximum frame time difference" );
	
	unsigned int initMinKeypoints;
	GetParamRequired( ph, "min_num_keypoints", initMinKeypoints );
	_minNumKeypoints.Initialize( ph, initMinKeypoints, "min_num_keypoints",
	                             "Minimum number of keyframe points." );
		
	std::string detectorType, trackerType, estimatorType;
	GetParamRequired( ph, "detector/type", detectorType );
	GetParamRequired( ph, "tracker/type", trackerType );
	GetParamRequired( ph, "estimator/type", estimatorType );

	ros::NodeHandle detectorHandle( "~detector" );
	if( detectorType == "corner" )
	{
		_detector = std::make_shared<CornerPointDetector>( _nodeHandle, detectorHandle );
	}
	else if( detectorType == "fixed" )
	{
		_detector = std::make_shared<FixedPointDetector>( _nodeHandle, detectorHandle );
	}
	else if( detectorType == "FAST" )
	{
		_detector = std::make_shared<FASTPointDetector>( _nodeHandle, detectorHandle );
	}
	else
	{
		ROS_ERROR_STREAM( "Invalid point detector type: " + detectorType );
	}
	
	ros::NodeHandle trackerHandle( "~tracker" );
	if( trackerType == "lucas_kanade" )
	{
		_tracker = std::make_shared<LKPointTracker>( _nodeHandle, trackerHandle );
	}
	else
	{
		ROS_ERROR_STREAM( "Invalid point tracker type." );
	}
	
	ros::NodeHandle estimatorHandle( "~estimator" );
	if( estimatorType == "rigid" )
	{
		_estimator = std::make_shared<RigidEstimator>( _nodeHandle, estimatorHandle );
	}
	else
	{
		ROS_ERROR_STREAM( "Invalid motion estimator type: " + estimatorType );
	}

	YAML::Node sources;
	GetParamRequired( ph, "sources", sources );
	YAML::const_iterator iter;
	for( iter = sources.begin(); iter != sources.end(); ++iter )
	{
		const std::string name = iter->first.as<std::string>();
		const YAML::Node& info = iter->second;;
		RegisterCamera( name, info );
	}
}

VisualOdometryPipeline::~VisualOdometryPipeline() {}

void VisualOdometryPipeline::RegisterCamera( const std::string& name, 
                                             const YAML::Node& info )
{
	ROS_INFO_STREAM( "Registering camera " << name );

	CameraRegistration& reg = _cameraRegistry[ name ];
	reg.name = name;
	reg.framesSkipped = 0;

	unsigned int buffSize;
	GetParam<unsigned int>( info, "buffer_size", buffSize, 1 );
	
	std::string imageTopic;
	GetParamRequired( info, "image_topic", imageTopic );
	if( _imageTopics.count( imageTopic ) > 0 )
	{
		ROS_WARN_STREAM( "Source: " << name << " image topic: " <<
		                 imageTopic << " already has subscriptions. Skipping subscribe step." );
	}
	else
	{
		reg.imageSub = _imagePort.subscribeCamera( imageTopic, 
		                                           buffSize, 
		               boost::bind( &VisualOdometryPipeline::ImageCallback, 
		                            this,
		                            boost::ref( reg ),
		                            _1,
		                            _2 ) );
		_imageTopics.insert( imageTopic );
	}

	GetParamRequired( info, "show_output", reg.showOutput );
	if( reg.showOutput )
	{
		std::string debugTopicName = _privHandle.resolveName( name + "/image_debug" );
		ROS_INFO_STREAM( "Displaying debug output for " << name << 
		                 " on topic" << debugTopicName );
		reg.debugPub = _imagePort.advertise( debugTopicName, 1 );
	}
	std::string velTopic;
	GetParamRequired( info, "output_topic", velTopic );
	reg.velPub = _nodeHandle.advertise<geometry_msgs::TwistStamped>( velTopic, 0 );
}

void VisualOdometryPipeline::ImageCallback( CameraRegistration& reg,
                                            const sensor_msgs::ImageConstPtr& msg,
                                            const sensor_msgs::CameraInfoConstPtr& info_msg )
{
	WriteLock lock( reg.mutex );

	FrameInterestPoints current;
	current.time = msg->header.stamp;
	camplex::CameraCalibration cc( "calib", *info_msg );
	// TODO A more complete check of intrinsics validity
	if( cc.GetFx() == 0.0 or std::isnan( cc.GetFx() ) )
	{
		ROS_WARN_STREAM( "Camera " << reg.name << " has invalid intrinsics." );
		return;
	}
	
	current.cameraModel = cc;
	cv::cvtColor( cv_bridge::toCvShare( msg )->image, current.frame, CV_BGR2GRAY );
	
	// Initialization catch
	double timeSinceLastFrame = ( current.time - reg.lastFrame.time ).toSec();
	if( reg.keyFrame.frame.empty() || 
		timeSinceLastFrame > _maxFrameDt ||
		reg.keyFrame.points.size() < _minNumKeypoints )
	{
		SetKeyframe( reg, current );
		if( reg.showOutput ) { VisualizeFrame( reg ); }
		return;
	}

	// Check for subsampling
	if( reg.framesSkipped < _subsampleRate )
	{
		reg.framesSkipped++;
		return;
	}
	reg.framesSkipped = 0;

	// Track interest points into current frame
	// TODO Move into a guess module or something?
	// Extract 2D pose from previou 3D pose
	MatrixType lastH = reg.lastPointsPose.ToMatrix();
	PoseSE2::Rotation lastR( 0 );
	lastR.fromRotationMatrix( lastH.block<2,2>( 1, 1 ) );
	Translation2Type lastT( lastH.block<2,1>( 1, 3 ) );
	PoseSE2 guessPose( lastT, lastR );

	// Since pose is in normalized coordniates, have to normalize, transform, then unnormalize again
	FrameInterestPoints keyFrameNorm = reg.keyFrame.Normalize();
	current.points = TransformPoints( keyFrameNorm.points, guessPose );
	current = current.Unnormalize();
	
	// ROS_INFO_STREAM( "Prev points: " << reg.lastFrame.points << std::endl <<
	                 // "Guess points: " << current.points );

	if( !_tracker->TrackInterestPoints( reg.keyFrame, 
	                                    current) )
	{
		ROS_INFO_STREAM( "Tracking failed! Resetting keyframe." );
		SetKeyframe( reg, current );
		if( reg.showOutput ) { VisualizeFrame( reg ); }
		return;
	}

	// Failure if not enough inliers in tracking
	size_t numTrackingInliers = current.points.size();
	double trackingInlierRatio = numTrackingInliers / (double) reg.originalNumKeypoints;
	if( trackingInlierRatio <= _minInlierRatio )
	{
		ROS_INFO_STREAM( numTrackingInliers << " inliers after tracking less than min " <<
		                 reg.originalNumKeypoints * _minInlierRatio << ". Resetting keyframe." );
		SetKeyframe( reg, current );
		if( reg.showOutput ) { VisualizeFrame( reg ); }
		return;
	}
	ROS_INFO_STREAM( "Post tracking inliers: " << numTrackingInliers );
	
	// Estimate motion between frames
	PoseSE3 currentPose;
	if( !_estimator->EstimateMotion( reg.keyFrame,
	                                 current,
	                                 currentPose ) )
	{
		ROS_WARN_STREAM( "Could not estimate motion between frames. Resetting keyframe." );
		SetKeyframe( reg, current );
		if( reg.showOutput ) { VisualizeFrame( reg ); }
		return;
	}

	// Check number of inliers
	size_t numMotionInliers = current.points.size();
	double motionInlierRatio = numMotionInliers / (double) numTrackingInliers;
	if( motionInlierRatio <= _minInlierRatio )
	{
		ROS_INFO_STREAM( numMotionInliers << " inliers after motion estimation less than "
		                 << _minInlierRatio * numTrackingInliers
		                 << ". Resetting keyframe." );
		SetKeyframe( reg, current );
		if( reg.showOutput ) { VisualizeFrame( reg ); }
		return;
	}
	ROS_INFO_STREAM( "Post estimation inliers: " << numMotionInliers );

	// Have to calculate dt before getting timestamp
	double dt = ( current.time - reg.lastFrame.time ).toSec();
	PoseSE3 cameraDisplacement = reg.lastPointsPose.Inverse() * currentPose;
	PoseSE3::TangentVector cameraVelocity = PoseSE3::Log( cameraDisplacement ) / dt;

	geometry_msgs::TwistStamped tmsg;
	tmsg.header = msg->header;
	tmsg.twist= TangentToMsg( cameraVelocity );
	reg.velPub.publish( tmsg );
	
	// Update registration
	reg.lastFrame = current;
	reg.lastPointsPose = currentPose;

	if( reg.showOutput ) { VisualizeFrame( reg ); }

	// Check if we need to redetect
	if( motionInlierRatio <= _redetectionThreshold )
	{
		// ROS_INFO_STREAM( reg.keyFrame.points.size() << " inliers less than "
		//                  << _redetectionThreshold * reg.originalNumKeypoints 
		//                  << ". Resetting keyframe." );
		SetKeyframe( reg, current );
		return;
	}
}
	
void VisualOdometryPipeline::VisualizeFrame( const CameraRegistration& reg )
{
	unsigned int width = reg.keyFrame.frame.cols;
	unsigned int height = reg.keyFrame.frame.rows;
	if( width == 0 || height == 0 )
	{
		ROS_WARN_STREAM( "VisualOdometryPipeline: Cannot visualize empty frame!" );
		return;
	}

	// Create image side-by-side
	cv::Mat visImage( height, width*2, reg.keyFrame.frame.type() );
	cv::Mat visLeft( visImage, cv::Rect( 0, 0, width, height ) );
	cv::Mat visRight( visImage, cv::Rect( width, 0, width, height ) );
	reg.keyFrame.frame.copyTo( visLeft );
	reg.lastFrame.frame.copyTo( visRight );
	
	// Display interest points
	cv::Point2d offset( width, 0 );
	for( unsigned int i = 0; i < reg.keyFrame.points.size(); i++ )
	{
		cv::circle( visLeft, reg.keyFrame.points[i], 3, cv::Scalar(0), -1, 8 );
		cv::circle( visLeft, reg.keyFrame.points[i], 2, cv::Scalar(255), -1, 8 );
	}
	for( unsigned int i = 0; i < reg.lastFrame.points.size(); i++ )
	{
		cv::circle( visRight, reg.lastFrame.points[i], 3, cv::Scalar(0), -1, 8 );
		cv::circle( visRight, reg.lastFrame.points[i], 2, cv::Scalar(255), -1, 8 );
	}
	
	std_msgs::Header header;
	header.stamp = reg.lastFrame.time;
	header.frame_id = reg.name;
	cv_bridge::CvImage vimg( header, "mono8", visImage );
	reg.debugPub.publish( vimg.toImageMsg() );
}

void VisualOdometryPipeline::SetKeyframe( CameraRegistration& reg,
                                          const FrameInterestPoints& key )
{
	// ROS_INFO_STREAM( "Setting keyframe..." );
	reg.keyFrame = key;
	reg.keyFrame.points = _detector->FindInterestPoints( reg.keyFrame.frame );
	reg.originalNumKeypoints = reg.keyFrame.points.size();
	if( reg.originalNumKeypoints < _minNumKeypoints )
	{
		ROS_INFO_STREAM( "Found " << reg.originalNumKeypoints << " keypoints, less than min: " <<
			             (unsigned int) _minNumKeypoints );
		reg.keyFrame.frame = cv::Mat();
		reg.keyFrame.points.clear();
		return;
	}

	// NOTE We want to keep them unnormalized for tracking purposes
	reg.keyFrame = reg.keyFrame.Undistort();

	reg.lastFrame = reg.keyFrame;

	reg.lastPointsPose = PoseSE3();
}

} // end namespace odoflow
