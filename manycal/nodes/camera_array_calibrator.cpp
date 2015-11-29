#include <ros/ros.h>

#include "manycal/CameraArrayCalibrator.h"

int main( int argc, char** argv )
{
	ros::init( argc, argv, "camera_array_calibrator" );
	
	ros::NodeHandle nh;
	ros::NodeHandle ph( "~" );
	
	manycal::CameraArrayCalibrator calibrator( nh, ph );
	
	while( ros::ok() )
	{
		ros::spinOnce();
	}
	
	calibrator.WriteResults();
	
	return 0;
}
