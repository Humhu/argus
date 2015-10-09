#include "atags/AtagDetector.h"

#include "apriltags/Tag16h5.h"
#include "apriltags/Tag25h7.h"
#include "apriltags/Tag25h9.h"
#include "apriltags/Tag36h9.h"
#include "apriltags/Tag36h11.h"

#include <cv_bridge/cv_bridge.h>

#include <opencv2/imgproc/imgproc.hpp>

#include <boost/foreach.hpp>

namespace atags
{
	
	AtagDetector::AtagDetector( const ros::NodeHandle& nh, const ros::NodeHandle& ph )
		: nodeHandle( nh ), 
		privHandle( ph ),
		imagePort( nh ),
		sequenceCounter( 0 )
	{

		int bufferLength;
		privHandle.param( "buffer_length", bufferLength, 3 );
		cameraSub = imagePort.subscribeCamera( "image_source", bufferLength, &AtagDetector::ImageCallback, this );
		
		rawPublisher = privHandle.advertise<argus_msgs::TagDetectionsStamped>( "detections_raw", 20 );
		rectifiedPublisher = privHandle.advertise<argus_msgs::TagDetectionsStamped>( "detections_rectified", 20 );
		
		privHandle.param<std::string>( "tag_family", tagFamily, "36h11" );
		if( tagFamily.compare( "16h5" ) == 0 )
		{
			detector = std::make_shared<AprilTags::TagDetector>( AprilTags::tagCodes16h5 );
		}
		else if( tagFamily.compare( "25h7" ) == 0 )
		{
			detector = std::make_shared<AprilTags::TagDetector>( AprilTags::tagCodes25h7 );
		}
		else if( tagFamily.compare( "25h9" ) == 0 )
		{
			detector = std::make_shared<AprilTags::TagDetector>( AprilTags::tagCodes25h9 );
		}
		else if( tagFamily.compare( "36h9" ) == 0 )
		{
			detector = std::make_shared<AprilTags::TagDetector>( AprilTags::tagCodes36h9 );
		}
		else if( tagFamily.compare( "36h11" ) == 0 )
		{
			detector = std::make_shared<AprilTags::TagDetector>( AprilTags::tagCodes36h11 );
		}
	}
	
	void AtagDetector::ImageCallback( const sensor_msgs::ImageConstPtr& msg,
									  const sensor_msgs::CameraInfoConstPtr& info_msg )
	{
		// Reinitializing the camera model is wasteful, but it may change over time
		image_geometry::PinholeCameraModel cameraModel;
		cameraModel.fromCameraInfo( info_msg );
		
		// Detection occurs in grayscale
		cv_bridge::CvImageConstPtr frame = cv_bridge::toCvShare( msg, "mono8" );
		std::vector<AprilTags::TagDetection> detections = detector->extractTags( frame->image );
		
		if( detections.size() == 0 ) { return; }
		
		std_msgs::Header header;
		header.frame_id = msg->header.frame_id;
		header.stamp = msg->header.stamp;
		argus_msgs::TagDetectionsStamped detMsg;
		detMsg.header = header;
		
		cv::Size imgSize = frame->image.size();
		
		// Publish raw
		PopulateMessage( detections, imgSize, detMsg, false );
		rawPublisher.publish( detMsg );
		
		// Checks for uncalibrated data
		if( cameraModel.fx() != 0 )
		{
			RectifyDetections( detections, cameraModel );
			
			// Publish rectified
			PopulateMessage( detections, imgSize, detMsg, true );
			rectifiedPublisher.publish( detMsg );
		}
		
	}
	
	void AtagDetector::PopulateMessage( const std::vector<AprilTags::TagDetection>& detections,
										const cv::Size& imgSize,
										argus_msgs::TagDetectionsStamped& msg,
										bool rectified ) 
	{
			msg.header.seq = sequenceCounter++;
			msg.sourceWidth = imgSize.width;
			msg.sourceHeight = imgSize.height;
			
			msg.detections.resize( detections.size() );
			
			for( unsigned int j = 0; j < detections.size(); j++ )
			{
				argus_msgs::TagDetection det;
				
				det.family = tagFamily;
				det.id = detections[j].id;
				det.hammingDistance = detections[j].hammingDistance;
				for( unsigned int i = 0; i < 4; i++ )
				{
					det.corners[i].x = detections[j].p[i].first;
					det.corners[i].y = detections[j].p[i].second;
				}
				det.center.x = detections[j].cxy.first;
				det.center.y = detections[j].cxy.second;
				det.normalized = rectified;
				msg.detections[j] = det;
			}
	}
	
	void AtagDetector::RectifyDetections( std::vector<AprilTags::TagDetection>& detections,
										  const image_geometry::PinholeCameraModel& cameraModel	)
	{
		cv::Matx33d cameraMatrix = cameraModel.intrinsicMatrix();
		cv::Mat distortionCoefficients = cameraModel.distortionCoeffs();
		
		cv::Mat detectedPoints( 5*detections.size(), 1, CV_64FC2 );
		
		unsigned int ind = 0;
		cv::Vec2d p;
		BOOST_FOREACH( AprilTags::TagDetection& det, detections )
		{
			for( unsigned int i = 0; i < 4; i++ )
			{
				p[0] = det.p[i].first;
				p[1] = det.p[i].second;
				detectedPoints.at<cv::Vec2d>(ind,0) = p;
				ind++;
			}
			p[0] = det.cxy.first;
			p[1] = det.cxy.second;
			detectedPoints.at<cv::Vec2d>(ind,0) = p;
			ind++;
		}
		
		cv::Mat undistortedPoints;
		cv::undistortPoints( detectedPoints, undistortedPoints, cameraMatrix,
							 distortionCoefficients, cv::noArray(), cv::noArray() );
		
		ind = 0;
		BOOST_FOREACH( AprilTags::TagDetection& det, detections )
		{
			for( unsigned int i = 0; i < 4; i++ )
			{
				p = undistortedPoints.at<cv::Vec2d>(ind,0);
				ind++;
				det.p[i].first = p[0];
				det.p[i].second = p[1];
			}
			p = undistortedPoints.at<cv::Vec2d>(ind,0);
			ind++;
			det.cxy.first = p[0];
			det.cxy.second = p[1];
		}
		
	}
	
}
