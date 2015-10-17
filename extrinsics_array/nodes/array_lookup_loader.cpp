#include <ros/ros.h>

#include <yaml-cpp/yaml.h>
#include "extrinsics_array/LookupInterface.h"

int main( int argc, char** argv )
{
	
	ros::init( argc, argv, "extrinsics_loader_node" );
	
	ros::NodeHandle nh;
	ros::NodeHandle ph( "~" );
	
	extrinsics_array::LookupInterface interface( nh );
	
	std::string lookupParent;
	if( !ph.getParam( "lookup_parent", lookupParent ) )
	{
		ROS_ERROR_STREAM( "No lookup parent specified." );
		return -1;
	}

	std::string lookupNamespace;
	ph.param <std::string> ( "lookup_namespace", lookupNamespace, "/lookup/" );
	ROS_INFO_STREAM( "Populating lookup at: " << lookupNamespace );
	interface.SetLookupNamespace( lookupNamespace );
	
	std::string calibPath;
	if( !ph.getParam( "calibration_path", calibPath ) )
	{
		ROS_ERROR_STREAM( "No calibration file specified." );
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
		return false;
	}
	
	// Populate lookup structure
	YAML::Node::const_iterator iter;
	for( iter = yaml.begin(); iter != yaml.end(); iter++ )
	{
		std::string name = iter->first.as< std::string >();
		interface.WriteLookup( name, lookupParent );
	}
	
	ROS_INFO_STREAM( "Loading complete." );
	return 0;
	
}
