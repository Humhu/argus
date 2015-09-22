#include "odoflow/VisualOdometryPipeline.h"
#include "odoflow/OpenCVMod.h"

#include "odoflow/CornerPointDetector.h"
#include "odoflow/FixedPointDetector.h"
#include "odoflow/FASTPointDetector.h"

#include "odoflow/LKPointTracker.h"

#include "odoflow/RigidEstimator.h"

#include <ros/ros.h>

#include <opencv2/highgui/highgui.hpp>

using namespace argus_utils;

namespace odoflow
{
	
VisualOdometryPipeline::VisualOdometryPipeline( ros::NodeHandle& nh, ros::NodeHandle& ph )
: nodeHandle( nh ), privHandle( ph ), redetectionThreshold( 10 ),
cameraMatrix( cv::Matx33d::eye() ), distortionCoefficients( cv::Mat::zeros( 4, 1, CV_64F ) ),
enableUndistortion( false )
{
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
	
}

VisualOdometryPipeline::~VisualOdometryPipeline()
{
	if( showOutput )
	{
		cv::destroyWindow( "Flow" );
	}
}

bool VisualOdometryPipeline::ProcessImage( const cv::Mat& image, 
											PoseSE3& displacement )
{
	
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
	if( enableUndistortion )
	{
		cv::Mat keyframeMat = ParsePointVector( keyframeInliers );
		cv::Mat procMat = ParsePointVector( procInliers );
		cv::Mat keyframeUndistortedMat, procUndistortedMat;
		cv::undistortPoints( keyframeMat, keyframeUndistortedMat, cameraMatrix,
							distortionCoefficients );
		cv::undistortPoints( procMat, procUndistortedMat, cameraMatrix,
							distortionCoefficients );
		keyframeTrackPoints = ParsePointMatrix( keyframeUndistortedMat );
		procTrackPoints = ParsePointMatrix( procUndistortedMat );
	}
	else
	{
		keyframeTrackPoints = keyframeInliers;
		procTrackPoints = procInliers;
	}
	
	// Use corresponding points to estimate the transform between the images
	bool estimateSuccess = 
		estimator->EstimateMotion( keyframeTrackPoints, procTrackPoints, displacement );
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
	
// 	if( showOutput )
// 	{
// 		cv::Mat visImage( image.rows, image.cols*2, image.type() );
// 		cv::Mat visLeft( visImage, cv::Rect( 0, 0, image.cols, image.rows) );
// 		cv::Mat visRight( visImage, cv::Rect( image.cols, 0, image.cols, image.rows ) );
// 		keyframe.copyTo( visLeft );
// 		image.copyTo( visRight );
// 		
// 		// Display interest points
// 		cv::Point2d offset( image.cols, 0 );
// 		for( unsigned int i = 0; i < keyframePoints.size(); i++ )
// 		{
// 			cv::circle( visLeft, keyframePoints[i], 3, cv::Scalar(0), -1, 8 );
// 			cv::circle( visLeft, keyframePoints[i], 2, cv::Scalar(255), -1, 8 );
// 		
// 			cv::circle( visRight, procInliers[i], 3, cv::Scalar(0), -1, 8 );
// 			cv::circle( visRight, procInliers[i], 2, cv::Scalar(255), -1, 8 );
// 			
// 			cv::Point2d start = keyframePoints[i];
// 			cv::Point2d finish = procInliers[i] + offset;
// 			cv::line( visImage, start, finish, cv::Scalar(255), 2 );
// 		}
// 		
// 		// Display linear velocity vector
// 		cv::Size imgSize = visImage.size();
// 		cv::Point2d imgCenter( imgSize.width/2, imgSize.height/2 );
// 		cv::Point2d arrowDisplacement( velocity(0), velocity(1) );
// 		cv::arrowedLine( visImage, imgCenter, imgCenter + arrowDisplacement, 
// 							cv::Scalar(0), 3 );
// 		cv::arrowedLine( visImage, imgCenter, imgCenter + arrowDisplacement, 
// 							cv::Scalar(255), 2 );
// 		
// 		// Display angular velocity
// 		int hudRadius = 15;
// 		double vw = velocity(5);
// 		cv::Point2f wIndicator( hudRadius*std::sin( vw ), hudRadius*std::cos( vw ) );
// 		cv::ellipse( visImage, imgCenter, cv::Size( hudRadius, hudRadius ), 90,
// 						0, vw*180/M_PI, cv::Scalar(0), 3 );
// 		cv::ellipse( visImage, imgCenter, cv::Size( hudRadius, hudRadius ), 90,
// 						0, vw*180/M_PI, cv::Scalar(255), 2 );
// 		
// 		cv::imshow( "Flow", visImage );
// 		cv::waitKey(1);
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

void VisualOdometryPipeline::SetRectificationParameters( const cv::Matx33d& cameraMat,
															const cv::Mat& distortionCoeffs )
{
	cameraMatrix = cameraMat;
	distortionCoefficients = distortionCoeffs;
}
	
} // end namespace odoflow
