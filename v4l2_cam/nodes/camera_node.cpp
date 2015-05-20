#include <ros/ros.h>

#include "v4l2_cam/DriverNode.h"

int main( int argc, char** argv )
{
	ros::init( argc, argv, "camera_driver" );
	
	ros::NodeHandle nh;
	ros::NodeHandle ph( "~" );
	
	v4l2_cam::DriverNode node( nh, ph, ros::this_node::getName() );
	
	ros::spin();
	return 0;
}
