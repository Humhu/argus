#include <ros/ros.h>

#include "v4l2_cam/CameraManager.h"

int main( int argc, char** argv )
{
	ros::init( argc, argv, "camera_manager" );
	
	ros::NodeHandle nh;
	ros::NodeHandle ph( "~" );
	
	v4l2_cam::CameraManager manager( nh, ph );
	
	ros::spin();
	
	return 0;
	
}
