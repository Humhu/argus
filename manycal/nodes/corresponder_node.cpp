#include <ros/ros.h>

#include "manycal/LatchCorresponder.h"
#include "manycal/TagCorrespondence.h"

#include "argus_msgs/TagDetection.h"

#include "v4l2_cam/CycleCameras.h"

#include <iostream>
#include <termios.h>

#include <boost/foreach.hpp>

using namespace argus;

int kbhit()
{
    struct timeval tv = { 0L, 0L };
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(0, &fds);
    return select(1, &fds, NULL, NULL, &tv);
}

int getch()
{
    int r;
    unsigned char c;
    if ((r = read(0, &c, sizeof(c))) < 0) {
        return r;
    } else {
        return c;
    }
}

class TagLatchCorresponder
	: public LatchCorresponder<argus_msgs::TagDetection, TagCorrespondence>
{
public:
	
	TagLatchCorresponder( const ros::NodeHandle& nh, const ros::NodeHandle& ph )
		: LatchCorresponder<argus_msgs::TagDetection, TagCorrespondence>( nh, ph )
	{}
		
protected:
	
	virtual void PublishLatch()
	{
		TagCorrespondence msg;
		msg.header = std_msgs::Header();
		BOOST_FOREACH( const argus_msgs::TagDetection::ConstPtr& det, latchedData )
		{
			argus_msgs::TagDetection dCopy( *det );
			msg.detections.push_back( dCopy );
		}
		// TODO Make more obvious that publisher is a member
		publisher.publish( msg );
	}
	
	ros::Subscriber subscriber;
	
};

int main( int argc, char** argv )
{
	
	ros::init( argc, argv, "detection_corresponder" );
	
	ros::NodeHandle nh;
	ros::NodeHandle ph( "~" );
	
	TagLatchCorresponder corresponder( nh, ph );
	
	std::string cameraManager;
	ph.getParam( "camera_manager", cameraManager );
	ros::ServiceClient cycleClient = nh.serviceClient<v4l2_cam::CycleCameras>(
		cameraManager + "/cycle_cameras", false );
	
	ros::AsyncSpinner spinner( 1 );
	spinner.start();
	
	ros::Rate r( 10.0 );
	
	while( ros::ok() )
	{
		int c = -1;
		if( kbhit() ) { c = getch(); }
		if( c == 'c' )
		{
			corresponder.StartLatch();
			
			// TODO Make blocking
			v4l2_cam::CycleCameras req;
			req.request.numToCapture = 1;
			req.request.blockUntilDone = true;
			cycleClient.call ( req );
		}
		r.sleep();
		
	}
	
	return 0;
}
