#include "odoflow/VisualOdometryPipeline.h"
#include "odoflow/OpenCVMod.h"
#include "odoflow/PoseSE3.h"

#include <opencv2/highgui/highgui.hpp>

namespace odoflow
{
	
	// TODO Automatically instantiate defaults?
	VisualOdometryPipeline::VisualOdometryPipeline()
		: showOutput( false ), redetectionThreshold( 10 ),
		cameraMatrix( cv::Matx33d::eye() )
	{}
	
	VisualOdometryPipeline::~VisualOdometryPipeline()
	{
		if( showOutput )
		{
			cv::destroyWindow( "Flow" );
		}
	}
	
	bool VisualOdometryPipeline::ProcessImage( const cv::Mat& image, 
											   const Timepoint& timestamp,
											   PoseSE3& displacement )
	{
		
		// If we don't have a key frame yet, take this as our key frame
		if( keyframe.empty() )
		{
			image.copyTo( keyframe );
			keyframeTime = timestamp;
			keyframePoints.clear();
		}
		
		// First identify interest points
		if( keyframePoints.size() < redetectionThreshold )
		{
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
		cv::Mat keyframeMat = ParsePointVector( keyframeInliers );
		cv::Mat procMat = ParsePointVector( procInliers );
		cv::Mat keyframeUndistortedMat, procUndistortedMat;
		cv::undistortPoints( keyframeMat, keyframeUndistortedMat, cameraMatrix,
							 distortionCoefficients );
		cv::undistortPoints( procMat, procUndistortedMat, cameraMatrix,
							 distortionCoefficients );
		InterestPoints keyframeUndistorted = ParsePointMatrix( keyframeUndistortedMat );
		InterestPoints procUndistorted = ParsePointMatrix( procUndistortedMat );
		
		// Use corresponding points to estimate the transform between the images
		bool estimateSuccess = 
			estimator->EstimateMotion( keyframeUndistorted, procUndistorted, displacement );
		
		TimeDuration timeDiff = timestamp - keyframeTime;
		double dt = timeDiff.total_microseconds()/((double) 1E6);
		
		PoseSE3::TangentVector velocity = PoseSE3::TangentVector::Zero();
		
		if( dt > 0 )
		{
			PoseSE3::TangentVector twist = se3log( displacement );
			velocity = twist/dt;
		}
		
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
			
				cv::circle( visRight, procInliers[i], 3, cv::Scalar(0), -1, 8 );
				cv::circle( visRight, procInliers[i], 2, cv::Scalar(255), -1, 8 );
				
				cv::Point2d start = keyframePoints[i];
				cv::Point2d finish = procInliers[i] + offset;
				cv::line( visImage, start, finish, cv::Scalar(255), 2 );
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
		
		if( estimateSuccess )
		{
			// Final loop bookkeeping
			keyframeTime = timestamp;
			image.copyTo( keyframe );
			keyframePoints = procInliers;
			midframePoints = procInliers;
		}
		else
		{
			midframePoints = procInliers;
		}
		
		return true;
	}
	
	void VisualOdometryPipeline::SetVisualization( bool enable )
	{
		showOutput = enable;
		if( showOutput )
		{
			cv::namedWindow( "Flow" );
		}
	}
	
	void VisualOdometryPipeline::SetRedetectionThreshold( unsigned int t )
	{
		redetectionThreshold = t;
	}
	
	void VisualOdometryPipeline::SetDetector( InterestPointDetector::Ptr det )
	{
		detector = det;
	}
	
	void VisualOdometryPipeline::SetTracker( InterestPointTracker::Ptr tr )
	{
		tracker = tr;
	}
	
	void VisualOdometryPipeline::SetEstimator( MotionEstimator::Ptr es )
	{
		estimator = es;
	}
	
	void VisualOdometryPipeline::SetRectificationParameters( const cv::Matx33d& cameraMat,
															 const cv::Mat& distortionCoeffs )
	{
		cameraMatrix = cameraMat;
		distortionCoefficients = distortionCoeffs;
	}
	
}
