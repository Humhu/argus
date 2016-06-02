#include "camera_array/RandomArrayManager.h"
#include "camera_array/PolicyManager.h"

using namespace argus;

int main( int argc, char** argv )
{
	ros::init( argc, argv, "camera_array_manager" );
	
	ros::NodeHandle nh;
	ros::NodeHandle ph( "~" );
	
	CameraArrayManager::Ptr manager;
	PolicyManager::Ptr policy;
	
	std::string managerType;
	ph.param<std::string>( "manager_type", managerType, "random" );
	
	if( managerType == "random" )
	{
		manager = std::make_shared<RandomArrayManager>( nh, ph );
	}
	else if( managerType == "greedy" )
	{
		manager = std::make_shared<CameraArrayManager>( nh, ph );
		policy = std::make_shared<PolicyManager>( nh, ph, manager );
	}
	else
	{
		ROS_ERROR_STREAM( "Invalid manager type." );
		return -1;
	}
		
	ros::spin();
	return 0;
}
