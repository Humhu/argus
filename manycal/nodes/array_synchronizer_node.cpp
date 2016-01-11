#include <ros/ros.h>

#include "manycal/ArraySynchronizer.h"

/*! \brief Node that cycles through a camera array and captures images. */
int main( int argc, char** argv )
{

	ros::init( argc, argv, "array_synchronizer" );
	
	ros::NodeHandle nodeHandle, privHandle( "~" );
	
	manycal::ArraySynchronizer cap( nodeHandle, privHandle );
	
	//	ros::AsyncSpinner spinner( 2 );
	// spinner.start();
	//ros::waitForShutdown();
	ros::spin();
	return 0;

	
}
