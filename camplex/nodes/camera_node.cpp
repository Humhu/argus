#include <ros/ros.h>

#include "camplex/DriverNode.h"

int main( int argc, char** argv )
{
	ros::init( argc, argv, "camera_driver" );
	
	ros::NodeHandle nh;
	ros::NodeHandle ph( "~" );
	
	camplex::DriverNode node( nh, ph );
	
	ros::spin();
	return 0;
}
