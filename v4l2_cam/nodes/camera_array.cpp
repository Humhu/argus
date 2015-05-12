#include <ros/ros.h>

#include "v4l2_cam/CameraArray.h"

int main( int argc, char** argv )
{
	ros::init( argc, argv, "camera_array" );
	
	ros::NodeHandle nh;
	ros::NodeHandle ph( "~" );
	
	v4l2_cam::CameraArray array( nh, ph );
	
	ros::spin();
	return 0;
}
