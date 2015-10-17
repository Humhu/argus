#include <ros/ros.h>

#include "fieldtrack/TargetTracker.h"

int main( int argc, char** argv )
{
	ros::init( argc, argv, "target_tracker" );
	
	ros::NodeHandle nh;
	ros::NodeHandle ph( "~" );
	
	fieldtrack::TargetTracker tracker( nh, ph );
	
	ros::spin();
	return 0;
}
