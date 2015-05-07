#include "atags/AtagDetector.h"
#include "atags/TagDetection.h"

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

		std::vector<std::string> topicNames;
		privHandle.getParam( "image_sources", topicNames );
		BOOST_FOREACH( const std::string& topic, topicNames )
		{
			cameraSub.push_back( imagePort.subscribeCamera( topic, 1, &AtagDetector::ImageCallback, this ) );
		}
		
		detectionPublisher = privHandle.advertise<atags::TagDetection>( "detections", 20 );
		
		std::string tagFamily;
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
		image_geometry::PinholeCameraModel cameraModel;
		cameraModel.fromCameraInfo( info_msg );
		
		cv_bridge::CvImageConstPtr frame = cv_bridge::toCvShare( msg, "mono8" );
		std::vector<AprilTags::TagDetection> detections =
			detector->extractTags( frame->image );
		
		if( detections.size() == 0 ) { return; }
		
		// Checks for uncalibrated data
		if( cameraModel.fx() != 0 )
		{
			RectifyDetections( detections, cameraModel );
		}
			
		std_msgs::Header header;
		header.frame_id = msg->header.frame_id;
		header.stamp = msg->header.stamp;
		BOOST_FOREACH( AprilTags::TagDetection& det, detections )
		{
			TagDetection detMsg;
 			detMsg.header = header;
			detMsg.header.seq = sequenceCounter++;
			detMsg.id = det.id;
			detMsg.hammingDistance = det.hammingDistance;
			for( unsigned int i = 0; i < 4; i++ )
			{
				detMsg.corners[i].x = det.p[i].first;
				detMsg.corners[i].y = det.p[i].second;
			}
			detectionPublisher.publish( detMsg );
		}
		
	}
	
	void AtagDetector::RectifyDetections( std::vector<AprilTags::TagDetection>& detections,
										  image_geometry::PinholeCameraModel& cameraModel	)
	{
		cv::Matx33d cameraMatrix = cameraModel.intrinsicMatrix();
		cv::Mat distortionCoefficients = cameraModel.distortionCoeffs();
		
		cv::Mat detectedPoints( 4*detections.size(), 1, CV_64FC2 );
		
		unsigned int ind = 0;
		BOOST_FOREACH( AprilTags::TagDetection& det, detections )
		{
			for( unsigned int i = 0; i < 4; i++ )
			{
				cv::Vec2d p;
				p[0] = det.p[i].first;
				p[1] = det.p[i].second;
				detectedPoints.at<cv::Vec2d>(ind,0) = p;
				ind++;
			}
		}
		
		cv::Mat undistortedPoints;
		cv::undistortPoints( detectedPoints, undistortedPoints, cameraMatrix,
							 distortionCoefficients, cv::noArray(), cv::noArray() );
		
		ind = 0;
		cv::Vec2d pt;
		BOOST_FOREACH( AprilTags::TagDetection& det, detections )
		{
			for( unsigned int i = 0; i < 4; i++ )
			{
				pt = undistortedPoints.at<cv::Vec2d>(ind,0);
				ind++;
				det.p[i].first = pt[0];
				det.p[i].second = pt[1];
			}
		}
		
	}
	
}
