#include "extrinsics_array/ExtrinsicsInfoManager.h"
#include "extrinsics_array/ExtrinsicsArrayCalibrationParsers.h"

#include "argus_utils/YamlUtils.h"

#include <boost/foreach.hpp>

using namespace argus_utils;

namespace extrinsics_array
{
	
ExtrinsicsInfoManager::ExtrinsicsInfoManager( ros::NodeHandle& nh )
: nodeHandle( nh ), lookupInterface( nodeHandle )
{}

ExtrinsicsInfoManager::~ExtrinsicsInfoManager() {}

void ExtrinsicsInfoManager::SetLookupNamespace( const std::string& loc )
{
	lookupInterface.SetLookupNamespace( loc );
}

bool ExtrinsicsInfoManager::ReadArrayInformation( const std::string& arrayPath, 
                                                  bool forceRead )
{
	if( !forceRead && failedArrayQueries.count( arrayPath ) > 0 ) { return false; }
	
	YAML::Node extrinsics;
	
	std::string p = SanitizeArrayPath( arrayPath );
	if( !GetYamlParam( nodeHandle, p, extrinsics ) )
	{
		ROS_WARN_STREAM( "Could not retrieve extrinsics array information at " << p );
		failedArrayQueries.insert( arrayPath );
		return false;
	}
	
	// Parse the array info into the map
	ExtrinsicsArrayInfo arrayInfo;
	if( !ParseExtrinsicsArrayCalibration( extrinsics, arrayInfo ) )
	{
		ROS_ERROR_STREAM( "Error parsing extrinsics information for " << p );
		failedArrayQueries.insert( arrayPath );
		return false;
	}
	arrayMap[ p ] = std::make_shared<ExtrinsicsArray>( arrayInfo );
	
	// Add the info to the extrinsics name map
	BOOST_FOREACH( const std::string& name, arrayInfo.memberNames )
	{
		memberMap[ name ] = p;
	}
	return true;
}

bool ExtrinsicsInfoManager::ReadMemberInformation( const std::string& memberName,
                                                   bool forceRead )
{
	if( !forceRead && failedMemberQueries.count( memberName ) > 0 ) { return false; }
	
	std::string arrayPath;
	if( !lookupInterface.ReadParent( memberName, arrayPath ) )
	{
		ROS_WARN_STREAM( "Could not find parent information for " + memberName );
		failedMemberQueries.insert( memberName );
		return false;
	}
	
	// Don't have to write to the memberMap because ReadArrayInformation will do it
	return ReadArrayInformation( arrayPath );
}

bool ExtrinsicsInfoManager::HasArray( const std::string& arrayPath ) const
{
	return arrayMap.count( SanitizeArrayPath( arrayPath ) ) != 0;
}

bool ExtrinsicsInfoManager::HasMember( const std::string& memberName ) const
{
	return memberMap.count( memberName ) != 0;
}

const ExtrinsicsArray& ExtrinsicsInfoManager::GetArray( const std::string& arrayPath )
{
	return *( arrayMap.at( arrayPath ) );
}
	
const ExtrinsicsArray& ExtrinsicsInfoManager::GetParentArray( const std::string& memberName )
{
	return *( arrayMap.at( memberMap.at( memberName ) ) );
}
	
const std::string& ExtrinsicsInfoManager::GetParentPath( const std::string& memberName )
{
	return memberMap.at( memberName );
}

std::string ExtrinsicsInfoManager::SanitizeArrayPath( const std::string& arrayPath )
{
	std::string p = arrayPath;
	if( p.back() != '/' ) { p += "/"; }
	return p;
}

} // end namespace extrinsics_array
