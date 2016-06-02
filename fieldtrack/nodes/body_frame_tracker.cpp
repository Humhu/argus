#include <ros/ros.h>

#include "fieldtrack/BodyFrameTracker.h"

int main( int argc, char** argv )
{
	ros::init( argc, argv, "body_frame_tracker" );
	
	ros::NodeHandle nh;
	ros::NodeHandle ph( "~" );
	
	argus::BodyFrameTracker tracker( nh, ph );
	
	ros::spin();
	return 0;
}
