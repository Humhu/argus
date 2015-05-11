#include <ros/ros.h>

#include "manycal/LatchCorresponder.h"
#include "manycal/TagCorrespondence.h"

#include "argus_msgs/TagDetection.h"

#include "v4l2_cam/CycleCameras.h"

#include <iostream>
#include <termios.h>

#include <boost/foreach.hpp>

using namespace manycal;

// TODO
int getch()
{
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);           // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON);                 // disable buffering      
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

  int c = getchar();  // read character (non-blocking)

  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
  return c;
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
		int c = getch();
		if( c == 'c' )
		{
			std::cout << "Starting latch..." << std::endl;
			corresponder.StartLatch();
			
			v4l2_cam::CycleCameras req;
			req.request.numToCapture = 1;
			std::cout << "Starting cycle..." << std::endl;
			cycleClient.call ( req );
			std::cout << "Cycle returned." << std::endl;
		}
		r.sleep();
		
	}
	
	std::cout << "Broke from loop." << std::endl;
	return 0;
}
