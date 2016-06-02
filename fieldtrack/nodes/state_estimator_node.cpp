#include <ros/ros.h>
#include "fieldtrack/SimpleStateEstimator.h"

int main( int argc, char** argv )
{
	ros::init( argc, argv, "state_estimator_node" );
	
	ros::NodeHandle nh;
	ros::NodeHandle ph( "~" );
	argus::SimpleStateEstimator estimator( nh, ph );
	
	ros::spin();
}
