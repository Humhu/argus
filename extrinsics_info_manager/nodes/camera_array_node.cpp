#include <ros/ros.h>

#include "camera_array/CameraArray.h"

int main( int argc, char** argv )
{
	ros::init( argc, argv, "camera_array" );
	
	ros::NodeHandle nh;
	ros::NodeHandle ph( "~" );
	
	camera_array::CameraArray array( nh, ph );
	
	ros::spin();
	return 0;
}
