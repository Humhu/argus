#include <ros/ros.h>

#include "atags/AtagDetector.h"

int main( int argc, char** argv )
{
	
	ros::init( argc, argv, "atag_detector" );
	
	ros::NodeHandle nh;
	ros::NodeHandle ph( "~" );
	
	atags::AtagDetector det( nh, ph );
	
	ros::spin();
	
	return 0;
	
}
