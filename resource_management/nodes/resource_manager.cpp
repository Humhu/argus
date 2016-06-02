#include <ros/ros.h>

#include "resource_management/ResourceManager.h"

int main( int argc, char** argv )
{
	ros::init( argc, argv, "resource_manager" );
	
	ros::NodeHandle nh;
	ros::NodeHandle ph( "~" );
	
	argus::ResourceManager manager( nh, ph );
	
	ros::spin();
	
	return 0;
}
