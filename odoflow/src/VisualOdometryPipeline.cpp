#include "odoflow/VisualOdometryPipeline.h"
#include "odoflow/OpenCVMod.h"

#include "odoflow/CornerPointDetector.h"
#include "odoflow/FixedPointDetector.h"
#include "odoflow/FASTPointDetector.h"

#include "odoflow/LKPointTracker.h"

#include "odoflow/RigidEstimator.h"

#include "argus_utils/YamlUtils.h"

#include <geometry_msgs/TwistStamped.h>
#include <image_geometry/pinhole_camera_model.h>

#include <opencv2/highgui/highgui.hpp>

using namespace argus_utils;

namespace odoflow
{
	
VisualOdometryPipeline::VisualOdometryPipeline( ros::NodeHandle& nh, ros::NodeHandle& ph )
: nodeHandle( nh ), privHandle( ph ), enableUndistortion( false ), imagePort( nodeHandle )
{
	XmlRpc::XmlRpcValue xml;
	if( ph.getParam( "camera_pose", xml ) )
	{
		YAML::Node yaml = XmlToYaml( xml );
		if( !GetPoseYaml( yaml, cameraPose ) )
		{
			ROS_ERROR_STREAM( "Could not parse camera pose." );
			exit( -1 );
		}
	}
	
	int rDet;
	ph.param( "redetection_threshold", rDet, 10 );
	redetectionThreshold = static_cast<unsigned int>( rDet );
	
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
	
	imageSub = imagePort.subscribeCamera( "/image_raw", 1, &VisualOdometryPipeline::ImageCallback, this );
	velPub = privHandle.advertise<geometry_msgs::TwistStamped>( "odometry", 10 );

}

VisualOdometryPipeline::~VisualOdometryPipeline()
{
	if( showOutput )
	{
		cv::destroyWindow( "Flow" );
	}
}

void VisualOdometryPipeline::ImageCallback( const sensor_msgs::ImageConstPtr& msg,
											const sensor_msgs::CameraInfoConstPtr& info_msg )
{
	camplex::CameraCalibration calib( "cam", *info_msg );
	
	ros::Time timestamp = msg->header.stamp;
	
	// Decode the received image into an OpenCV object
	cv_bridge::CvImageConstPtr frame;
	try
	{
		frame = cv_bridge::toCvShare( msg, "mono8" );
	}
	catch( cv_bridge::Exception& e )
	{
		ROS_ERROR( "VisualOdometryNode cv_bridge exception: %s", e.what() );
		return;
	}
	
	cv::Mat image = frame->image;
	
	PoseSE3 displacement;
	bool success = ProcessImage( image, calib, displacement );
	
	double dt = timestamp.toSec() - lastTime.toSec();
	if( success && dt > 0.0 )
	{
		
		PoseSE3::TangentVector cTangent = se3log( displacement );
		PoseSE3::TangentVector velocity = cameraPose.Adjoint( cTangent / dt );
		
		geometry_msgs::TwistStamped msg;
		msg.header.stamp = timestamp;
		msg.twist.linear.x = velocity(0);
		msg.twist.linear.y = velocity(1);
		msg.twist.linear.z = velocity(2);
		msg.twist.angular.x = velocity(3);
		msg.twist.angular.y = velocity(4);
		msg.twist.angular.z = velocity(5);
		
		velPub.publish( msg );
		
		if( showOutput )
		{
			cv::Mat visImage( image.rows, image.cols*2, image.type() );
			cv::Mat visLeft( visImage, cv::Rect( 0, 0, image.cols, image.rows) );
			cv::Mat visRight( visImage, cv::Rect( image.cols, 0, image.cols, image.rows ) );
			keyframe.copyTo( visLeft );
			image.copyTo( visRight );
			
			// Display interest points
			cv::Point2d offset( image.cols, 0 );
			for( unsigned int i = 0; i < keyframePoints.size(); i++ )
			{
				cv::circle( visLeft, keyframePoints[i], 3, cv::Scalar(0), -1, 8 );
				cv::circle( visLeft, keyframePoints[i], 2, cv::Scalar(255), -1, 8 );
			
// 				cv::circle( visRight, procInliers[i], 3, cv::Scalar(0), -1, 8 );
// 				cv::circle( visRight, procInliers[i], 2, cv::Scalar(255), -1, 8 );
				
// 				cv::Point2d start = keyframePoints[i];
// 				cv::Point2d finish = procInliers[i] + offset;
// 				cv::line( visImage, start, finish, cv::Scalar(255), 2 );
			}
			
			// Display linear velocity vector
			cv::Size imgSize = visImage.size();
			cv::Point2d imgCenter( imgSize.width/2, imgSize.height/2 );
			cv::Point2d arrowDisplacement( velocity(0), velocity(1) );
			cv::arrowedLine( visImage, imgCenter, imgCenter + arrowDisplacement, 
								cv::Scalar(0), 3 );
			cv::arrowedLine( visImage, imgCenter, imgCenter + arrowDisplacement, 
								cv::Scalar(255), 2 );
			
			// Display angular velocity
			int hudRadius = 15;
			double vw = velocity(5);
			cv::Point2f wIndicator( hudRadius*std::sin( vw ), hudRadius*std::cos( vw ) );
			cv::ellipse( visImage, imgCenter, cv::Size( hudRadius, hudRadius ), 90,
							0, vw*180/M_PI, cv::Scalar(0), 3 );
			cv::ellipse( visImage, imgCenter, cv::Size( hudRadius, hudRadius ), 90,
							0, vw*180/M_PI, cv::Scalar(255), 2 );
			
			cv::imshow( "Flow", visImage );
			cv::waitKey(1);
		}
		
	}
	
	lastTime = timestamp;
}

bool VisualOdometryPipeline::ProcessImage( const cv::Mat& image, 
										   camplex::CameraCalibration calibration,
										   PoseSE3& displacement )
{
	
	calibration.SetScale( image.size() );
	image_geometry::PinholeCameraModel cameraModel;
	cameraModel.fromCameraInfo( calibration.GetInfo() );
	
	// If we don't have a key frame yet, take this as our key frame
	if( keyframe.empty() )
	{
		image.copyTo( keyframe );
		keyframePoints.clear();
	}
	
	// First identify interest points
	if( keyframePoints.size() < redetectionThreshold )
	{
		ROS_DEBUG( "Redetecting keyframe points." );
		keyframePoints = detector->FindInterestPoints( keyframe );
		midframePoints = keyframePoints;
	}
	
	// Find corresponding points in next image
	// TODO Catch when the detector may fail to find any interest points?
	InterestPoints keyframeInliers, procInliers;
	std::vector<bool> trackMask;
	tracker->TrackInterestPoints( keyframe, image, keyframePoints, midframePoints,
									trackMask, keyframeInliers, procInliers );
	keyframePoints = keyframeInliers;
	
	// Rectify the points to normalized image coordinates
	InterestPoints keyframeTrackPoints, procTrackPoints;
// 	if( enableUndistortion )
// 	{
		cv::Mat keyframeMat = ParsePointVector( keyframeInliers );
		cv::Mat procMat = ParsePointVector( procInliers );
		cv::Mat keyframeUndistortedMat, procUndistortedMat;
		cv::undistortPoints( keyframeMat, keyframeUndistortedMat, cameraModel.intrinsicMatrix(),
							cameraModel.distortionCoeffs() );
		cv::undistortPoints( procMat, procUndistortedMat, cameraModel.intrinsicMatrix(),
							cameraModel.distortionCoeffs() );
		keyframeTrackPoints = ParsePointMatrix( keyframeUndistortedMat );
		procTrackPoints = ParsePointMatrix( procUndistortedMat );
// 	}
// 	else
// 	{
// 		keyframeTrackPoints = keyframeInliers;
// 		procTrackPoints = procInliers;
// 	}
	
	// Use corresponding points to estimate the transform between the images
	bool estimateSuccess = 
		estimator->EstimateMotion( procTrackPoints, keyframeTrackPoints, displacement );
	if( !estimateSuccess )
	{
		ROS_WARN( "Failed to estimate motion from frames." );
	}
		
// 	PoseSE3::TangentVector velocity = PoseSE3::TangentVector::Zero();
	
// 	if( dt > 0 )
// 	{
// 		PoseSE3::TangentVector twist = se3log( displacement );
// 		velocity = twist/dt;
// 	}
	
	// NOTE Currently if we have a tracking failure we get lost forever
	// TODO If we want to keep midframe tracking we may need to force an upper
	// bound on number of midframes to avoid this problem
// 		if( estimateSuccess )
// 		{
		// Final loop bookkeeping
		image.copyTo( keyframe );
		keyframePoints = procInliers;
		midframePoints = procInliers;
// 		}
// 		else
// 		{
// 			midframePoints = procInliers;
// 		}
	
	return true;
}

} // end namespace odoflow
