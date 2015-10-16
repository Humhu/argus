#include "extrinsics_array/ExtrinsicsLoader.h"
#include "argus_utils/YamlUtils.h"
#include <boost/foreach.hpp>

using namespace argus_utils;

namespace extrinsics_array
{

ExtrinsicsLoader::ExtrinsicsLoader( ros::NodeHandle& nh )
: nodeHandle( nh ), lookupInterface( nh )
{}

void ExtrinsicsLoader::SetLookupNamespace( const std::string& ns )
{
	lookupInterface.SetLookupNamespace( ns );
}

void ExtrinsicsLoader::SetLookupParent( const std::string& parent )
{
	parentPath = parent;
}

bool ExtrinsicsLoader::LoadYAML( const std::string& filePath, const std::string& paramPath )
{
	YAML::Node yaml;
	try
	{
		yaml = YAML::LoadFile( filePath );
	}
	catch( YAML::BadFile e )
	{
		ROS_ERROR_STREAM( "Could not read YAML file at " << filePath );
		return false;
	}
	
	// Write loaded parameters to server
	XmlRpc::XmlRpcValue xml = YamlToXml( yaml );
	nodeHandle.setParam( paramPath, xml ); 
	
	// Populate lookup structure
	YAML::Node::const_iterator iter;
	for( iter = yaml.begin(); iter != yaml.end(); iter++ )
	{
		std::string name = iter->first.as< std::string >();
		lookupInterface.WriteLookup( name, parentPath );
	}
	
	return true;
}
	
} // end namespace extrinsics_array 
