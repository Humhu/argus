#include "fiducial_array/FiducialArrayInfoManager.h"

int main( int argc, char** argv )
{
	
	ros::init( argc, argv, "fiducial_array_info_manager" );
	
	ros::NodeHandle nh; 
	ros::NodeHandle ph( "~" );
	
	fiducial_array::FiducialArrayInfoManager manager( nh, ph );
	
	std::string arrayName;
	if( ph.getParam( "array_name", arrayName ) )
	{
		manager.SetReferenceName( arrayName );
	}
	
	std::string calibrationPath;
	if( ph.getParam( "array_info_url", calibrationPath ) )
	{
		manager.LoadFiducialArrayInfo( calibrationPath );
	}
	else
	{
		ROS_INFO_STREAM( "No calibration file specified. Array will be uncalibrated." );
	}
	
	ros::spin();
	exit( 0 );
}
	
	
