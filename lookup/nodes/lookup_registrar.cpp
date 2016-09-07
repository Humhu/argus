#include <ros/ros.h>

#include <yaml-cpp/yaml.h>
#include "argus_utils/utils/YamlUtils.h"
#include "lookup/LookupInterface.h"

using namespace argus;

/*! \brief This node fully resolves a topic and registers it with the lookup service. */
int main( int argc, char** argv )
{
	
	ros::init( argc, argv, "lookup_registrar" );
	
	ros::NodeHandle nh;
	ros::NodeHandle ph( "~" );
	LookupInterface lookupInterface;
	
	std::string lookupNamespace;
	ph.param<std::string>( "lookup_namespace", lookupNamespace, "/lookup/" );
	if( lookupNamespace.back() != '/' ) 
	{ 
		lookupNamespace += "/"; 
		
	}
	ROS_INFO_STREAM( "Using lookup namespace: " << lookupNamespace );
	lookupInterface.SetLookupNamespace( lookupNamespace );
	
	std::string targetName;
	if( !ph.getParam( "target_name", targetName ) )
	{
		ROS_ERROR_STREAM( "Must specify the target name." );
		return -1;
	}
	
	std::string targetNamespace;
	ph.param<std::string>( "target_namespace", targetNamespace, "" );
	
	lookupInterface.WriteNamespace( targetName, targetNamespace );
	
	bool keepParams;
	ph.param( "keep_params", keepParams, false );
	if( !keepParams )
	{
		ph.deleteParam("");
	}
	
	return 0;
	
}
