#include "camera_array/RandomArrayManager.h"

int main( int argc, char** argv )
{
	ros::init( argc, argv, "camera_array_manager" );
	
	ros::NodeHandle nh;
	ros::NodeHandle ph( "~" );
	
	camera_array::CameraArrayManager::Ptr manager;
	
	std::string managerType;
	ph.param<std::string>( "manager_type", managerType, "random" );
	
	if( managerType == "random" )
	{
		manager = std::make_shared<camera_array::RandomArrayManager>( nh, ph );
	}
	else
	{
		ROS_ERROR_STREAM( "Invalid manager type." );
		return -1;
	}
		
	ros::spin();
	return 0;
}
