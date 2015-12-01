#include "odoflow/VisualOdometryPipeline.h"
#include "odoflow/OpenCVMod.h"

#include "odoflow/CornerPointDetector.h"
#include "odoflow/FixedPointDetector.h"
#include "odoflow/FASTPointDetector.h"

#include "odoflow/LKPointTracker.h"

#include "odoflow/RigidEstimator.h"

#include "argus_utils/GeometryUtils.h"
#include "argus_utils/MatrixUtils.h"
#include "argus_utils/ParamUtils.h"

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <opencv2/highgui/highgui.hpp>

using namespace argus_utils;

namespace odoflow
{
	
VisualOdometryPipeline::VisualOdometryPipeline( ros::NodeHandle& nh, ros::NodeHandle& ph )
: nodeHandle( nh ), privHandle( ph ), imagePort( nodeHandle ), 
extrinsicsManager( lookupInterface )
{
	
	std::string lookupNamespace;
	privHandle.param<std::string>( "lookup_namespace", lookupNamespace, "/lookup" );
	lookupInterface.SetLookupNamespace( lookupNamespace );
	
	GetParamDefault<unsigned int>( ph, "redetection_threshold", redetectionThreshold, 10 );
	GetParamDefault<unsigned int>( ph, "min_num_inliers", minNumInliers, 10 );
	
	std::vector<double> covarianceData;
	if( ph.getParam( "covariance_rate", covarianceData ) )
	{
		if( !ParseMatrix( covarianceData, covarianceRate ) )
		{
			ROS_ERROR_STREAM( "Could not parse covariance rate matrix." );
			exit( -1 );
		}
	}
	else
	{
		covarianceRate = PoseSE3::CovarianceMatrix::Identity();
	}
		
	std::string detectorType, trackerType, estimatorType;
	ph.param<std::string>( "detector/type", detectorType, "corner" );
	ph.param<std::string>( "tracker/type", trackerType, "lucas_kanade" );
	ph.param<std::string>( "estimator/type", estimatorType, "rigid" );
	
	if( detectorType == "corner" )
	{
		detector = std::make_shared<CornerPointDetector>( nodeHandle, privHandle );
	}
	else if( detectorType == "fixed" )
	{
		detector = std::make_shared<FixedPointDetector>( nodeHandle, privHandle );
	}
	else if( detectorType == "FAST" )
	{
		detector = std::make_shared<FASTPointDetector>( nodeHandle, privHandle );
	}
	else
	{
		ROS_ERROR_STREAM( "Invalid point detector type." );
	}
	
	if( trackerType == "lucas_kanade" )
	{
		tracker = std::make_shared<LKPointTracker>( nodeHandle, privHandle );
	}
	else
	{
		ROS_ERROR_STREAM( "Invalid point tracker type." );
	}
	
	if( estimatorType == "rigid" )
	{
		estimator = std::make_shared<RigidEstimator>( nodeHandle, privHandle );
	}
	else
	{
		ROS_ERROR_STREAM( "Invalid motion estimator type." );
	}
	
	ph.param( "show_output", showOutput, false );
	if( showOutput )
	{
		debugPub = imagePort.advertise( "image_debug", 1 );
	}
	imageSub = imagePort.subscribeCamera( "image", 1, &VisualOdometryPipeline::ImageCallback, this );
	dispPub = nodeHandle.advertise<geometry_msgs::PoseWithCovarianceStamped>( "displacements", 10 );

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
	
	// Get the camera registration
	const std::string& cameraName = msg->header.frame_id;
	if( cameraRegistry.count( cameraName ) == 0 ) { 
		if( !RegisterCamera( cameraName ) )
		{
			ROS_WARN_STREAM( "Could not register source camera " << cameraName );
			return;
		}
	}
	CameraRegistration& registration = cameraRegistry[ cameraName ];
	cv::Mat currentFrame = cv_bridge::toCvShare( msg, "mono8" )->image;
	ros::Time now = msg->header.stamp;
	
	// Initialization catch
	if( registration.keyframe.empty() )
	{
		SetKeyframe( registration, currentFrame, cameraModel, now );
		return;
	}
	
	// Track interest points into current frame
	InterestPoints keyframeInliersImage, currentInliersImage;
	tracker->TrackInterestPoints( registration.keyframe, currentFrame, 
	                              registration.keyframePointsImage, registration.lastPointsImage,
	                              keyframeInliersImage, currentInliersImage );
	
	// Failure if not enough inliers in tracking
	size_t numInliers = keyframeInliersImage.size();
	if( numInliers < minNumInliers )
	{
		ROS_WARN_STREAM( "Tracked " << numInliers << " inliers, less than min " << minNumInliers );
		SetKeyframe( registration, currentFrame, cameraModel, now );
		return;
	}
	ROS_INFO_STREAM( "Tracked " << numInliers << " inliers." );
	
	// Undistort and normalize tracked inliers
	InterestPoints currentPointsImage, currentPointsNormalized;
	UndistortPoints( currentInliersImage, cameraModel, true, false, currentPointsImage );
	UndistortPoints( currentPointsImage, cameraModel, false, true, currentPointsNormalized );
	
	// TODO Assuming camera model does not change here to normalize keyframe inliers
	// Also normalize points for pose estimation
	InterestPoints keyframePointsNormalized;
	UndistortPoints( keyframeInliersImage, cameraModel, false, true, 
	                 keyframePointsNormalized );
	

	// Estimate motion between frames
	PoseSE3 currentPose;
	if( !estimator->EstimateMotion( currentPointsNormalized, keyframePointsNormalized, currentPose ) )
	{
		ROS_WARN_STREAM( "Could not estimate motion between frames. Resetting keyframe." );
		SetKeyframe( registration, currentFrame, cameraModel, now );
		return;
	}
	registration.keyframePointsImage = keyframeInliersImage;
	
	// Have to calculate dt before getting timestamp
	double dt = ( now - registration.lastPointsTimestamp ).toSec();
        //PoseSE3 cameraDisplacement = registration.lastPointsPose.Inverse() * currentPose;
	const PoseSE3& cameraExtrinsics = extrinsicsManager.GetExtrinsics( cameraName );
	PoseSE3 framePose = currentPose * cameraExtrinsics.Inverse();
	PoseSE3 frameDisplacement = registration.lastPointsPose.Inverse() * framePose;
	//PoseSE3 frameDisplacement = cameraPose * cameraDisplacement * cameraPose.Inverse();
	//ROS_INFO_STREAM( "Camera displacement for " << cameraName << ": " << cameraDisplacement );
	ROS_INFO_STREAM( "Frame displacement for " << cameraName << ": " << frameDisplacement );
	
	geometry_msgs::PoseWithCovarianceStamped tmsg;
	tmsg.header.stamp = now;
	tmsg.header.frame_id = extrinsicsManager.GetReferenceFrame( cameraName );
	tmsg.pose.pose= PoseToMsg( frameDisplacement );
	SerializeMatrix( covarianceRate * dt, tmsg.pose.covariance );
	dispPub.publish( tmsg );
	
	registration.lastPointsTimestamp = now;
	registration.lastPointsImage = currentInliersImage;
	registration.lastPointsPose = framePose; //currentPose;
	
	// If we don't have enough inlier interest points, redetect on our keyframe
	if( registration.keyframePointsImage.size() < redetectionThreshold )
	{
		ROS_INFO_STREAM( "Not enough keyframe points. Resetting keyframe." );
		SetKeyframe( registration, currentFrame, cameraModel, now );
	}
	
	if( showOutput ) { VisualizeFrame( registration, currentFrame, now ); }
}
	
void VisualOdometryPipeline::VisualizeFrame( const CameraRegistration& registration, 
                                             const cv::Mat& frame,
                                             const ros::Time& timestamp )
{
	
	// Create image side-by-side
	cv::Mat visImage( frame.rows, frame.cols*2, frame.type() );
	cv::Mat visLeft( visImage, cv::Rect( 0, 0, frame.cols, frame.rows) );
	cv::Mat visRight( visImage, cv::Rect( frame.cols, 0, frame.cols, frame.rows ) );
	registration.keyframe.copyTo( visLeft );
	frame.copyTo( visRight );
	
	// Display interest points
	cv::Point2d offset( frame.cols, 0 );
	for( unsigned int i = 0; i < registration.keyframePointsImage.size(); i++ )
	{
		cv::circle( visLeft, registration.keyframePointsImage[i], 3, cv::Scalar(0), -1, 8 );
		cv::circle( visLeft, registration.keyframePointsImage[i], 2, cv::Scalar(255), -1, 8 );
	}
	for( unsigned int i = 0; i < registration.lastPointsImage.size(); i++ )
	{
		cv::circle( visRight, registration.lastPointsImage[i], 3, cv::Scalar(0), -1, 8 );
		cv::circle( visRight, registration.lastPointsImage[i], 2, cv::Scalar(255), -1, 8 );
	}
	
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
	header.frame_id = registration.name;
	cv_bridge::CvImage vimg( header, "mono8", visImage );
	debugPub.publish( vimg.toImageMsg() );
}
	

bool VisualOdometryPipeline::RegisterCamera( const std::string& name )
{
	if( cameraRegistry.count( name ) > 0 ) { return true; }
	
	if( !extrinsicsManager.HasMember( name ) )
	{
		if( !extrinsicsManager.ReadMemberInformation( name, false ) ) { return false; }
	}
	
	CameraRegistration registration;
	registration.name = name;
	cameraRegistry[ name ] = registration;
	return true;
}

void VisualOdometryPipeline::SetKeyframe( CameraRegistration& registration,
                                          const cv::Mat& frame,
                                          const image_geometry::PinholeCameraModel& model,
                                          const ros::Time& timestamp )
{
	ROS_INFO_STREAM( "Setting keyframe and detecting interest points" );
	
	registration.keyframe = frame;
	registration.keyframeTimestamp = timestamp;
	registration.lastPointsImage.clear();
	registration.lastPointsTimestamp = timestamp;
	registration.lastPointsPose = extrinsicsManager.GetExtrinsics( registration.name ).Inverse(); //PoseSE3();
	InterestPoints detected = detector->FindInterestPoints( registration.keyframe );
	UndistortPoints( detected, model, true, false, registration.keyframePointsImage );
	ROS_INFO_STREAM( "Found " << registration.keyframePointsImage.size() << " points." );	
}

} // end namespace odoflow
