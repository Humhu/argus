#include <ros/ros.h>

#include <yaml-cpp/yaml.h>
#include "argus_utils/YamlUtils.h"
#include "lookup/LookupInterface.h"

using namespace argus_utils;

static std::string lookupNamespace;

void ProcessNode( const std::string& root, const YAML::Node& node, ros::NodeHandle& nh )
{
	YAML::Node::const_iterator iter;
	for( iter = node.begin(); iter != node.end(); iter++ )
	{
		std::string key = iter->first.as<std::string>();
		std::string subroot = root + key + "/";
		YAML::Node contents = iter->second;
		
		if( !contents.IsMap() ) 
		{
			ROS_ERROR_STREAM( "Incorrectly formed YAML file." );
			exit( -1 );
		}
		
		// If node is not terminal, recursively process it
		if( !contents["lookup_value"] )
		{
			ProcessNode( subroot, iter->second, nh );
			continue;
		}
		
		// Else process the terminal node
		std::string lookupValue = contents[ "lookup_value" ].as<std::string>();
		if( contents["resolve_value"] )
		{
			if( contents["resolve_value"].as<bool>() )
			{
				lookupValue = nh.resolveName( lookupValue );
			}
		}
		ROS_INFO_STREAM( "Registering value (" << lookupValue << ") to key ("
		    << subroot << ") with lookup (" << lookupNamespace << ")" );

		// TODO Move into LookupInterface?
		nh.setParam( lookupNamespace + subroot, lookupValue );
	}
}

/*! \brief This node loads files contents and writes registration information to
 * a global lookup service. 
 * Parameters:
 *   file_path: Full path to a YAML file to be parsed and loaded to the namespace
 *              containing the node. Each top-level YAML node is treated as a 
 *              separate object to lookup-register. */
int main( int argc, char** argv )
{
	
	ros::init( argc, argv, "lookup_file_registrar" );
	
	ros::NodeHandle nh;
	ros::NodeHandle ph( "~" );
	
	ph.param<std::string>( "lookup_namespace", lookupNamespace, "/lookup/" );
	if( lookupNamespace.back() != '/' ) { lookupNamespace += "/"; }
	
	std::string calibPath;
	if( !ph.getParam( "file_path", calibPath ) )
	{
		ROS_ERROR_STREAM( "Must specify file_path." );
		return -1;
	}
	
	YAML::Node yaml;
	try
	{
		yaml = YAML::LoadFile( calibPath );
	}
	catch( YAML::BadFile e )
	{
		ROS_ERROR_STREAM( "Could not read YAML file at " << calibPath );
		return -1;
	}
	
	ProcessNode( "", yaml, nh );
	
	ROS_INFO_STREAM( "Loading complete." );
	return 0;
	
}
