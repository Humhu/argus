#include <ros/ros.h>
#include "fieldtrack/VelocityIntegrator.h"

int main( int argc, char** argv )
{
	ros::init( argc, argv, "velocity_integrator" );
	
	ros::NodeHandle nh, ph( "~" );
	
	argus::VelocityIntegrator tracker( nh, ph );
	
	ros::spin();
	return 0;
}
