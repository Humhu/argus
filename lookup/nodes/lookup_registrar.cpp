#include <ros/ros.h>

#include <yaml-cpp/yaml.h>
#include "argus_utils/utils/ParamUtils.h"
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
	
	YAML::Node targets;
	GetParamRequired( ph, "", targets );
	YAML::Node::const_iterator iter;
	for( iter = targets.begin(); iter != targets.end(); iter++ )
	{
		const std::string& name = iter->first.as<std::string>();
		const std::string& target = iter->second.as<std::string>();
		ROS_INFO_STREAM( "Writing entry for " << name << " routing to " << target );
		lookupInterface.WriteNamespace( name, target );
	}
	
	return 0;
	
}
