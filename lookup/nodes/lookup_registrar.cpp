#include <ros/ros.h>

#include <yaml-cpp/yaml.h>
#include "argus_utils/YamlUtils.h"
#include "lookup/LookupInterface.h"

using namespace argus_utils;

/*! \brief This node fully resolves a topic and registers it with the lookup service. */
int main( int argc, char** argv )
{
	
	ros::init( argc, argv, "lookup_registrar" );
	
	ros::NodeHandle nh;
	ros::NodeHandle ph( "~" );
	
	std::string lookupNamespace;
	ph.param<std::string>( "lookup_namespace", lookupNamespace, "/lookup/" );
	if( lookupNamespace.back() != '/' ) 
	{ 
		lookupNamespace += "/"; 
		
	}
	
	std::string lookupKey;
	if( !ph.getParam( "lookup_key", lookupKey ) )
	{
		ROS_ERROR_STREAM( "Must specify the key to register the topic to." );
		return -1;
	}
	if( lookupKey.front() == '/' ) 
	{ 
		lookupKey.erase( lookupKey.begin() ); 
	}
	
	std::string lookupValue;
	if( !ph.getParam( "lookup_value", lookupValue ) )
	{
		ROS_ERROR_STREAM( "Must specify the value to register." );
		return -1;
	}
	
	bool resolve;
	ph.param( "resolve_value", resolve, false );
	if( resolve )
	{
		lookupValue = nh.resolveName( lookupValue );
	}
	
	ROS_INFO_STREAM( "Registering topic (" << lookupValue << ") to key ("
	    << lookupKey << ") with lookup (" << lookupNamespace << ")" );
	
	nh.setParam( lookupNamespace + lookupKey, lookupValue );
	
	ROS_INFO_STREAM( "Registration complete." );
	
	return 0;
	
}
