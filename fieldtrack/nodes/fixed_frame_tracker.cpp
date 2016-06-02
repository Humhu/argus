#include <ros/ros.h>

#include "fieldtrack/FixedFrameTracker.h"

int main( int argc, char** argv )
{
	ros::init( argc, argv, "fixed_frame_tracker" );
	
	ros::NodeHandle nh;
	ros::NodeHandle ph( "~" );
	
	argus::FixedFrameTracker tracker( nh, ph );
	
	ros::spin();
	return 0;
}
