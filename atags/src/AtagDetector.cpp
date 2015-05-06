#include "atags/AtagDetector.h"
#include "atags/TagDetection.h"

#include "apriltags/Tag16h5.h"
#include "apriltags/Tag25h7.h"
#include "apriltags/Tag25h9.h"
#include "apriltags/Tag36h9.h"
#include "apriltags/Tag36h11.h"

#include <cv_bridge/cv_bridge.h>

#include <boost/foreach.hpp>

namespace atags
{
	
	AtagDetector::AtagDetector( const ros::NodeHandle& nh, const ros::NodeHandle& ph )
		: nodeHandle( nh ), 
		privHandle( ph ),
		imagePort( nh )
	{

		std::string topicName = nodeHandle.resolveName( "image_raw" );
		cameraSub = imagePort.subscribeCamera( topicName, 1, &AtagDetector::ImageCallback, this );
		
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
		cameraModel.fromCameraInfo( info_msg );
		// TODO Rectification
		
		cv_bridge::CvImageConstPtr frame = cv_bridge::toCvShare( msg, "mono8" );
		std::vector<AprilTags::TagDetection> detections =
			detector->extractTags( frame->image );
			
		BOOST_FOREACH( const AprilTags::TagDetection& det, detections )
		{
			TagDetection detMsg;
			detMsg.header = msg->header;
			detMsg.code = det.code;
			detMsg.id = det.id;
			detMsg.hammingDistance = det.hammingDistance;
			for( unsigned int i = 0; i < 4; i++ )
			{
				detMsg.corners[i].x = det.p[i].first;
				detMsg.corners[i].y = det.p[i].second;
			}
			detMsg.center.x = det.cxy.first;
			detMsg.center.y = det.cxy.second;
			detectionPublisher.publish( detMsg );
		}
		
	}
	
}
