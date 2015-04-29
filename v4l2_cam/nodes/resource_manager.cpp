#include <ros/ros.h>

#include "v4l2_cam/ResourceManager.h"

int main( int argc, char** argv )
{
	ros::init( argc, argv, "resource_manager" );
	
	ros::NodeHandle nh;
	ros::NodeHandle ph( "~" );
	
	v4l2_cam::ResourceManager manager( nh, ph );
	
	ros::spin();
	
	return 0;
}
