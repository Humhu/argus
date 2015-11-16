#include "extrinsics_array/ExtrinsicsInfoManager.h"
#include "extrinsics_array/ExtrinsicsArrayCalibrationParsers.h"

#include "argus_utils/YamlUtils.h"

#include <boost/foreach.hpp>

using namespace argus_utils;

namespace extrinsics_array
{
	
ExtrinsicsInfoManager::ExtrinsicsInfoManager( lookup::LookupInterface& interface )
: lookupInterface( interface )
{}

bool ExtrinsicsInfoManager::ReadMemberInformation( const std::string& memberName,
                                                   bool forceRead )
{
	// Fast-fail using cache
	if( !forceRead && failedQueries.count( memberName ) > 0 ) { return false; }
	
	std::string memberNamespace;
	if( !lookupInterface.ReadNamespace( memberName, memberNamespace ) )
	{
		ROS_WARN_STREAM( "Could not find namespace for: " << memberName );
		failedQueries.insert( memberName );
		return false;
	}

	MemberRegistration registration;
	
	// Read extrinsics first
	YAML::Node extrinsics;
	std::string extrinsicsKey = memberNamespace + "extrinsics";
	if( !GetYamlParam( nodeHandle, extrinsicsKey, extrinsics ) )
	{
		ROS_WARN_STREAM( "Could not find extrinsics information for: " << memberName 
		    << " at path " << extrinsicsKey );
		return false;
	}
	if( !GetPoseYaml( extrinsics, registration.extrinsics ) )
	{
		ROS_WARN_STREAM( "Could not parse extrinsics information for: " << memberName
		    << " at key: " << extrinsicsKey );
		return false;
	}
	
	// Then read reference frame ID
	YAML::Node reference;
	std::string referenceKey = memberNamespace + "frame_id";
	if( !GetYamlParam( nodeHandle, referenceKey, reference ) )
	{
		ROS_WARN_STREAM( "Could not find reference frame information for: " << memberName
		    << " at path " << referenceKey );
		return false;
	}
	registration.frameID = reference.as<std::string>();
	memberRegistry[ memberName ] = registration;
	return true;
}

bool ExtrinsicsInfoManager::HasMember( const std::string& memberName ) const
{
	return memberRegistry.count( memberName ) != 0;
}

const PoseSE3& ExtrinsicsInfoManager::GetExtrinsics( const std::string& memberName ) const
{
	return memberRegistry.at( memberName ).extrinsics;
}

const std::string& ExtrinsicsInfoManager::GetReferenceFrame( const std::string& memberName ) const
{
	return memberRegistry.at( memberName ).frameID;
}

} // end namespace extrinsics_array
