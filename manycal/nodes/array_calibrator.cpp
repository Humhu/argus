#include <ros/ros.h>

#include "manycal/ArrayCalibrator.h"

int main( int argc, char** argv )
{
	ros::init( argc, argv, "array_calibrator" );
	
	ros::NodeHandle nh;
	ros::NodeHandle ph( "~" );
	
	manycal::ArrayCalibrator calibrator( nh, ph );
	
	ros::spin();
	
	return 0;
}
