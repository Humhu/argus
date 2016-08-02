#include "odoflow/VisualOdometryPipeline.h"
#include "odoflow/OpenCVMod.h"

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
	
	unsigned int initRedectThresh;
	GetParamRequired( ph, "redetection_threshold", initRedectThresh );
	_redetectionThreshold.Initialize( ph, initRedectThresh, "redetection_threshold", 
	                                  "Pipeline feature redetection min threshold" );
	_redetectionThreshold.AddCheck<GreaterThan>( 0 );

	unsigned int initMinInlierNum;
	GetParamRequired( ph, "min_inlier_num", initMinInlierNum );
	_minNumInliers.Initialize( ph, initMinInlierNum, "min_inlier_num",
	                           "Pipeline min feature inlier threshold" );
	_minNumInliers.AddCheck<GreaterThan>( 0 );
	
	
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
	GetParam<std::string>( ph, "detector/type", detectorType, "corner" );
	GetParam<std::string>( ph, "tracker/type", trackerType, "lucas_kanade" );
	GetParam<std::string>( ph, "estimator/type", estimatorType, "rigid" );

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
	
	ph.param( "show_output", _showOutput, false );
	if( _showOutput )
	{
		_debugPub = _imagePort.advertise( "image_debug", 1 );
	}

	unsigned int buffSize;
	GetParam<unsigned int>( ph, "image_buffer_size", buffSize, 1 );
	_imageSub = _imagePort.subscribeCamera( "image", buffSize, &VisualOdometryPipeline::ImageCallback, this );
	_dispPub = _nodeHandle.advertise<geometry_msgs::TwistWithCovarianceStamped>( "velocity", 10 );

}

VisualOdometryPipeline::~VisualOdometryPipeline() {}

// TODO Redetection
void VisualOdometryPipeline::ImageCallback( const sensor_msgs::ImageConstPtr& msg,
                                            const sensor_msgs::CameraInfoConstPtr& info_msg )
{
	image_geometry::PinholeCameraModel cameraModel;
	cameraModel.fromCameraInfo( *info_msg );
	if( std::isnan( cameraModel.fx() ) ) 
	{
		ROS_ERROR_STREAM( "Received uncalibrated data." );
		return;
	}
	
	// Get the camera reg
	const std::string& cameraName = msg->header.frame_id;
	if( _cameraRegistry.count( cameraName ) == 0 ) { 
		if( !RegisterCamera( cameraName ) )
		{
			ROS_WARN_STREAM( "Could not register source camera " << cameraName );
			return;
		}
	}
	CameraRegistration& reg = _cameraRegistry[ cameraName ];
	cv::Mat msgFrame = cv_bridge::toCvShare( msg )->image;
	cv::Mat currentFrame;
	cv::cvtColor( msgFrame, currentFrame, CV_BGR2GRAY );
	ros::Time now = msg->header.stamp;
	
	// Initialization catch
	if( reg.keyframe.empty() )
	{
	  ROS_DEBUG_STREAM( "Initializing keyframe." );
		SetKeyframe( reg, currentFrame, cameraModel, now );
		return;
	}

	// Predict point motion
	// InterestPoints predictedPoints;
	double dt = ( msg->header.stamp - reg.lastPointsTimestamp ).toSec();
	// PoseSE2 disp = PoseSE2::Exp( -reg.lastVelocity*dt );
	// std::cout << "vel: " << reg.lastVelocity.transpose() << std::endl;
	// std::cout << "displacement: " << disp << std::endl;
	// TransformPoints( reg.lastPointsImage, disp, predictedPoints );
	// reg.lastPointsPredicted = predictedPoints;

	// for( unsigned int i = 0; i < reg.lastPointsImage.size(); ++i )
	// {
	// 	std::cout << "prev: " << reg.lastPointsImage[i].x << ", " << reg.lastPointsImage[i].y << std::endl;
	// 	std::cout << "est: " << predictedPoints[i].x << ", " << predictedPoints[i].y << std::endl;
	// }

	// Track interest points into current frame
	InterestPoints keyframeInliersImage, currentInliersImage;
	ROS_DEBUG_STREAM( "Keyframe points size: " << reg.keyframePointsImage.size() );
	ROS_DEBUG_STREAM( "Last points size: " << reg.lastPointsImage.size() );
	_tracker->TrackInterestPoints( reg.keyframe, currentFrame, 
	                              reg.keyframePointsImage, reg.lastPointsImage,
	                              keyframeInliersImage, currentInliersImage );
	
	// Failure if not enough inliers in tracking
	size_t numInliers = keyframeInliersImage.size();
	if( numInliers < _minNumInliers )
	{
		ROS_WARN_STREAM( "Tracked " << numInliers << " inliers, less than min " << _minNumInliers );
		SetKeyframe( reg, currentFrame, cameraModel, now );
		return;
	}
	ROS_DEBUG_STREAM( "Tracked " << numInliers << " inliers." );
	
	// Undistort and normalize tracked inliers
	InterestPoints currentPointsImage, currentPointsNormalized;
	UndistortPoints( currentInliersImage, cameraModel, true, false, currentPointsImage );
	//currentPointsImage = currentInliersImage;
	UndistortPoints( currentPointsImage, cameraModel, false, true, currentPointsNormalized );
	
	// TODO Assuming camera model does not change here to normalize keyframe inliers
	// Also normalize points for pose estimation
	InterestPoints keyframePointsNormalized;
	UndistortPoints( keyframeInliersImage, cameraModel, false, true, 
	                 keyframePointsNormalized );
	

	// Estimate motion between frames
	PoseSE3 currentPose;
	PoseSE2 pixPose;
	std::vector<uchar> motionInliers;
	if( !_estimator->EstimateMotion( currentPointsNormalized, keyframePointsNormalized, 
	                                motionInliers, currentPose, pixPose ) )
	{
		ROS_WARN_STREAM( "Could not estimate motion between frames. Resetting keyframe." );
		SetKeyframe( reg, currentFrame, cameraModel, now );
		return;
	}
	
	InterestPoints currentMotionInliers, keyframeMotionInliers;
	currentMotionInliers.reserve( currentInliersImage.size() );
	keyframeMotionInliers.reserve( keyframeInliersImage.size() );
	for( unsigned int i = 0; i < currentPointsNormalized.size(); i++ )
	{
		if( !motionInliers[i] ) { continue; }
		keyframeMotionInliers.push_back( keyframeInliersImage[i] );
		currentMotionInliers.push_back( currentInliersImage[i] );
	}
	reg.keyframePointsImage = keyframeMotionInliers;
	ROS_DEBUG_STREAM( "Kept " << currentMotionInliers.size() << " inliers after motion estimation." );
	
	// Have to calculate dt before getting timestamp
    PoseSE3 cameraDisplacement = reg.lastPointsPose.Inverse() * currentPose;
	const ExtrinsicsInfo& cameraInfo = _extrinsicsManager.GetInfo( cameraName );
	PoseSE3::TangentVector cameraVelocity = PoseSE3::Log( cameraDisplacement ) / dt;
	PoseSE3::TangentVector frameVelocity = PoseSE3::Adjoint( cameraInfo.extrinsics ) * cameraVelocity;
	// ROS_DEBUG_STREAM( "Camera velocity " << cameraName << ": " << cameraVelocity.transpose() );
	// ROS_DEBUG_STREAM( "Frame velocity " << cameraName << ": " << frameVelocity.transpose() );

	// camplex::CameraCalibration cal( "", *info_msg );
	// MatrixType intrinsics = MatrixType::Identity(3,3);
	// intrinsics(0,0) = cal.GetFx();
	// intrinsics(1,1) = cal.GetFy();
	// intrinsics(0,2) = cal.GetCx();
	// intrinsics(1,2) = cal.GetCy();
	// std::cout << "normalized pixel pose: " << pixPose << std::endl;
	// MatrixType p = pixPose.ToTransform().matrix() * intrinsics;
	// PoseSE2 pp( p );
	// std::cout << "raw pixel pose: " << pp << std::endl;
	// std::cout << "last pixel pose: " << reg.lastPixelsPose << std::endl;
	// PoseSE2 pixelDisplacement = reg.lastPixelsPose.Inverse() * pp;
	// reg.lastVelocity = PoseSE2::Log( pixelDisplacement ) / dt;
	// std::cout << "pixel vel: " << reg.lastVelocity.transpose() << std::endl;
	// reg.lastPixelsPose = pp;

	geometry_msgs::TwistWithCovarianceStamped tmsg;
	tmsg.header.stamp = now;
	tmsg.header.frame_id = cameraInfo.referenceFrame;
	tmsg.twist.twist= TangentToMsg( frameVelocity );
	SerializeMatrix( _obsCovariance, tmsg.twist.covariance );
	_dispPub.publish( tmsg );
	
	// Update registration
	reg.lastPointsTimestamp = now;
	reg.lastPointsImage = currentMotionInliers;
	reg.lastPointsPose = currentPose;

	if( _showOutput ) { VisualizeFrame( reg, currentFrame, now ); }
	
	// If we don't have enough inlier interest points, redetect on our keyframe
	if( reg.keyframePointsImage.size() < _redetectionThreshold )
	{
		ROS_DEBUG_STREAM( "Not enough keyframe points. Resetting keyframe." );
		SetKeyframe( reg, currentFrame, cameraModel, now );
	}
       
}
	
void VisualOdometryPipeline::VisualizeFrame( const CameraRegistration& reg, 
                                             const cv::Mat& frame,
                                             const ros::Time& timestamp )
{
	// Create image side-by-side
	cv::Mat visImage( frame.rows, frame.cols*2, frame.type() );
	cv::Mat visLeft( visImage, cv::Rect( 0, 0, frame.cols, frame.rows) );
	cv::Mat visRight( visImage, cv::Rect( frame.cols, 0, frame.cols, frame.rows ) );
	reg.keyframe.copyTo( visLeft );
	frame.copyTo( visRight );
	
	// Display interest points
	cv::Point2d offset( frame.cols, 0 );
	for( unsigned int i = 0; i < reg.keyframePointsImage.size(); i++ )
	{
		cv::circle( visLeft, reg.keyframePointsImage[i], 3, cv::Scalar(0), -1, 8 );
		cv::circle( visLeft, reg.keyframePointsImage[i], 2, cv::Scalar(255), -1, 8 );
	}
	for( unsigned int i = 0; i < reg.lastPointsImage.size(); i++ )
	{
		cv::circle( visRight, reg.lastPointsImage[i], 3, cv::Scalar(0), -1, 8 );
		cv::circle( visRight, reg.lastPointsImage[i], 2, cv::Scalar(255), -1, 8 );
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
	header.stamp = timestamp;
	header.frame_id = reg.name;
	cv_bridge::CvImage vimg( header, "mono8", visImage );
	_debugPub.publish( vimg.toImageMsg() );
}
	

bool VisualOdometryPipeline::RegisterCamera( const std::string& name )
{
	if( _cameraRegistry.count( name ) > 0 ) { return true; }
	
	if( !_extrinsicsManager.HasMember( name ) )
	{
		if( !_extrinsicsManager.ReadMemberInfo( name, false ) ) { return false; }
	}
	
	CameraRegistration reg;
	reg.name = name;
	_cameraRegistry[ name ] = reg;
	return true;
}

void VisualOdometryPipeline::SetKeyframe( CameraRegistration& reg,
                                          const cv::Mat& frame,
                                          const image_geometry::PinholeCameraModel& model,
                                          const ros::Time& timestamp )
{
	ROS_DEBUG_STREAM( "Setting keyframe and detecting interest points" );
	
	reg.keyframe = frame;
	reg.keyframeTimestamp = timestamp;
	reg.lastPointsImage.clear();
	//reg.lastDelta = InterestPoint( 0, 0 );
	reg.lastPointsTimestamp = timestamp;
	reg.lastPointsPose = PoseSE3(); //__extrinsicsManager.GetExtrinsics( reg.name ).Inverse(); //PoseSE3();
	InterestPoints detected = _detector->FindInterestPoints( reg.keyframe );
	UndistortPoints( detected, model, true, false, reg.keyframePointsImage );
	//reg.keyframePointsImage = detected;
	ROS_DEBUG_STREAM( "Found " << reg.keyframePointsImage.size() << " points." );	
}

} // end namespace odoflow
