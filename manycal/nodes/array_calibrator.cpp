#include <ros/ros.h>

#include "manycal/ArrayCalibrator.h"

int main( int argc, char** argv )
{
	ros::init( argc, argv, "array_calibrator" );
	
	ros::NodeHandle nh;
	ros::NodeHandle ph( "~" );
	
	argus::ArrayCalibrator calibrator( nh, ph );
	
	ros::MultiThreadedSpinner spinner(2);
	spinner.spin();

	return 0;
}
