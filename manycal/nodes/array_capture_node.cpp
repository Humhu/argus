#include <ros/ros.h>

#include "manycal/ArrayCapturer.h"

/*! \brief Node that cycles through a camera array and captures images. */
int main( int argc, char** argv )
{

	ros::init( argc, argv, "array_capturer" );
	
	ros::NodeHandle nodeHandle, privHandle( "~" );
	
	manycal::ArrayCapturer cap( nodeHandle, privHandle );
	
	ros::AsyncSpinner spinner( 2 );
	spinner.start();
	ros::waitForShutdown();
	
}
