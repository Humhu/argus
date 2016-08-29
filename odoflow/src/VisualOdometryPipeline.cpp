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

#include <geometry_msgs/TwistWithCovarianceStamped.h>

#include <opencv2/highgui/highgui.hpp>

namespace argus
{

VisualOdometryPipeline::VisualOdometryPipeline( ros::NodeHandle& nh, ros::NodeHandle& ph )
: _nodeHandle( nh ), 
_privHandle( ph ), 
_imagePort( _nodeHandle ), 
_extrinsicsManager( _lookupInterface )
{
	std::string lookupNamespace;
	_privHandle.param<std::string>( "lookup_namespace", lookupNamespace, "/lookup" );
	_lookupInterface.SetLookupNamespace( lookupNamespace );
	
	std::string featureName;
	GetParam<std::string>( ph, "feature_name", featureName, "vo_features" );
	std::vector<std::string> featureDescriptions = { "inlier_ratio" };
	_featureTx.InitializePushStream( featureName, 
	                                 ph,
	                                 1,
	                                 featureDescriptions );

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
	
	std::vector<double> covarianceData;
	if( ph.getParam( "velocity_covariance", covarianceData ) )
	{
		if( !ParseMatrix( covarianceData, _obsCovariance ) )
		{
			ROS_ERROR_STREAM( "Could not parse observation covariance matrix." );
			exit( -1 );
		}
	}
	else
	{
		_obsCovariance = PoseSE3::CovarianceMatrix::Identity();
	}
		
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
		ROS_INFO_STREAM( "Registering camera " << name );

		if( !_extrinsicsManager.HasMember( name ) )
		{
			if( !_extrinsicsManager.ReadMemberInfo( name, false ) ) 
			{ 
				throw std::runtime_error( "VisualOdometryPipeline: Could not retrieve extrinsics for " 
				                          + name );
			}
		}

		CameraRegistration& reg = _cameraRegistry[ name ];
		reg.name = name;

		const YAML::Node& info = iter->second;;
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
			                                           &VisualOdometryPipeline::ImageCallback, 
			                                           this );
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
		reg.velPub = _nodeHandle.advertise<geometry_msgs::TwistWithCovarianceStamped>( velTopic, 0 );
	}
}

VisualOdometryPipeline::~VisualOdometryPipeline() {}

void VisualOdometryPipeline::ImageCallback( const sensor_msgs::ImageConstPtr& msg,
                                            const sensor_msgs::CameraInfoConstPtr& info_msg )
{
	// Get the camera reg
	const std::string& cameraName = msg->header.frame_id;
	if( _cameraRegistry.count( cameraName ) == 0 )
	{
		// TODO Printout?
		return;
	}

	CameraRegistration& reg = _cameraRegistry[ cameraName ];
	WriteLock lock( reg.mutex );

	FrameInterestPoints current;
	current.time = msg->header.stamp;
	current.cameraModel = camplex::CameraCalibration( "calib", *info_msg );
	cv::cvtColor( cv_bridge::toCvShare( msg )->image, current.frame, CV_BGR2GRAY );
	
	// Initialization catch
	if( reg.keyFrame.frame.empty() )
	{
		SetKeyframe( reg, current );
		return;
	}

	// Track interest points into current frame
	// TODO Initialize a better guess
	current.points = reg.lastFrame.points;
	if( !_tracker->TrackInterestPoints( reg.keyFrame, 
	                                    current) )
	{
		ROS_INFO_STREAM( "Tracking failed! Resetting keyframe." );
		SetKeyframe( reg, current );
		if( reg.showOutput ) { VisualizeFrame( reg ); }
		return;
	}

	// Failure if not enough inliers in tracking
	double inlierRatio = current.points.size() / (double) reg.originalNumKeypoints;
	if( inlierRatio <= _minInlierRatio )
	{
		ROS_INFO_STREAM( current.points.size() << " inliers after tracking less than min " <<
		                 reg.originalNumKeypoints * _minInlierRatio << ". Resetting keyframe." );
		SetKeyframe( reg, current );
		if( reg.showOutput ) { VisualizeFrame( reg ); }
		return;
	}
	
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
	double keypointRatio = reg.keyFrame.points.size() / (double) reg.originalNumKeypoints;
	if( keypointRatio <= _minInlierRatio )
	{
		ROS_INFO_STREAM( reg.keyFrame.points.size() << " inliers after motion estimation less than "
		                 << _minInlierRatio * reg.originalNumKeypoints 
		                 << ". Resetting keyframe." );
		SetKeyframe( reg, current );
		if( reg.showOutput ) { VisualizeFrame( reg ); }
		return;
	}

	// Have to calculate dt before getting timestamp
	double dt = ( current.time - reg.lastFrame.time ).toSec();
	PoseSE3 cameraDisplacement = reg.lastPointsPose.Inverse() * currentPose;
	const ExtrinsicsInfo& cameraInfo = _extrinsicsManager.GetInfo( cameraName );
	PoseSE3::TangentVector cameraVelocity = PoseSE3::Log( cameraDisplacement ) / dt;
	PoseSE3::TangentVector frameVelocity = PoseSE3::Adjoint( cameraInfo.extrinsics ) * cameraVelocity;
	// ROS_INFO_STREAM( "dt: " << dt << std::endl <<
	//                  "displacement: " << cameraDisplacement << std::endl <<
	//                  "cam velocity: " << cameraVelocity.transpose() );

	geometry_msgs::TwistWithCovarianceStamped tmsg;
	tmsg.header.stamp = current.time;
	tmsg.header.frame_id = cameraInfo.referenceFrame;
	tmsg.twist.twist= TangentToMsg( frameVelocity );
	SerializeMatrix( _obsCovariance, tmsg.twist.covariance );
	reg.velPub.publish( tmsg );
	
	// Update registration
	reg.lastFrame = current;
	reg.lastPointsPose = currentPose;

	if( reg.showOutput ) { VisualizeFrame( reg ); }

	// Check if we need to redetect
	if( keypointRatio <= _redetectionThreshold )
	{
		ROS_INFO_STREAM( reg.keyFrame.points.size() << " inliers less than "
		                 << _redetectionThreshold * reg.originalNumKeypoints 
		                 << ". Resetting keyframe." );
		SetKeyframe( reg, current );
		return;
	}
}
	
void VisualOdometryPipeline::VisualizeFrame( const CameraRegistration& reg )
{
	unsigned int width = reg.keyFrame.frame.cols;
	unsigned int height = reg.keyFrame.frame.rows;

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
	// for( unsigned int i = 0; i < reg.lastPointsPredicted.size(); i++ )
	// {
	// 	cv::circle( visRight, reg.lastPointsPredicted[i], 3, cv::Scalar(0), -1, 8 );
	// 	cv::circle( visRight, reg.lastPointsPredicted[i], 2, cv::Scalar(128), -1, 8 );
	// }
	
	// TODO?
	// Display linear velocity vector
// 	cv::Size imgSize = visImage.size();
// 	cv::Point2d imgCenter( imgSize.width/2, imgSize.height/2 );
// 	cv::Point2d arrowDisplacement( w(0), w(1) );
// 	cv::arrowedLine( visImage, imgCenter, imgCenter + arrowDisplacement, 
// 						cv::Scalar(0), 3 );
// 	cv::arrowedLine( visImage, imgCenter, imgCenter + arrowDisplacement, 
// 						cv::Scalar(255), 2 );
	
	// Display angular velocity
// 	int hudRadius = 15;
// 	double vw = w(5);
// 	cv::Point2f wIndicator( hudRadius*std::sin( vw ), hudRadius*std::cos( vw ) );
// 	cv::ellipse( visImage, imgCenter, cv::Size( hudRadius, hudRadius ), 90,
// 					0, vw*180/M_PI, cv::Scalar(0), 3 );
// 	cv::ellipse( visImage, imgCenter, cv::Size( hudRadius, hudRadius ), 90,
// 					0, vw*180/M_PI, cv::Scalar(255), 2 );
	
	std_msgs::Header header;
	header.stamp = reg.lastFrame.time;
	header.frame_id = reg.name;
	cv_bridge::CvImage vimg( header, "mono8", visImage );
	reg.debugPub.publish( vimg.toImageMsg() );
}

void VisualOdometryPipeline::SetKeyframe( CameraRegistration& reg,
                                          const FrameInterestPoints& key )
{
	reg.keyFrame = key;
	reg.keyFrame.points = _detector->FindInterestPoints( reg.keyFrame.frame );
	// NOTE We want to keep them unnormalized for tracking purposes
	reg.keyFrame = reg.keyFrame.Undistort();

	reg.lastFrame = reg.keyFrame;

	reg.lastPointsPose = PoseSE3();
	reg.originalNumKeypoints = reg.keyFrame.points.size();
}

} // end namespace odoflow
