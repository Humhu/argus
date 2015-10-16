#include <ros/ros.h>

#include "fieldtrack/TrackerManager.h"

int main( int argc, char** argv )
{
	ros::init( argc, argv, "target_tracker" );
	
	ros::NodeHandle nh;
	ros::NodeHandle ph( "~" );
	
	fieldtrack::TrackerManager tracker( nh, ph );
	
	ros::spin();
	return 0;
}
