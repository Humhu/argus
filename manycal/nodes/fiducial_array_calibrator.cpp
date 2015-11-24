#include <ros/ros.h>

#include "manycal/FiducialArrayCalibrator.h"

int main( int argc, char** argv )
{
	ros::init( argc, argv, "fiducial_array_calibrator" );
	
	ros::NodeHandle nh;
	ros::NodeHandle ph( "~" );
	
	manycal::FiducialArrayCalibrator calibrator( nh, ph );
	
	while( ros::ok() )
	{
		ros::spinOnce();
	}
	
	calibrator.PrintResults();
	
	return 0;
}
