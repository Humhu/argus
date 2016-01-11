#include "extrinsics_array/ExtrinsicsInfoManager.h"

#include "argus_utils/YamlUtils.h"
#include "argus_utils/ParamUtils.h"

#include <boost/foreach.hpp>

using namespace argus_utils;

namespace extrinsics_array
{
	
ExtrinsicsInfoManager::ExtrinsicsInfoManager( lookup::LookupInterface& interface )
: InfoManager( interface )
{}

bool ExtrinsicsInfoManager::ReadMemberInfo( const std::string& memberName,
                                            bool forceLookup,
	                                        const ros::Duration& timeout )
{
	std::string memberNamespace;
	if( !GetNamespace( memberName, memberNamespace, forceLookup, timeout ) ) 
	{ 
		return false; 
	}

	ExtrinsicsInfo registration;
	
	// Read extrinsics first
	YAML::Node extrinsics;
	std::string extrinsicsKey = GenerateExtrinsicsKey( memberNamespace );
	if( !GetYamlParam( nodeHandle, extrinsicsKey, extrinsics ) )
	{
		ROS_WARN_STREAM( "Could not find extrinsics information for: " << memberName 
		    << " at path " << extrinsicsKey );
		RecordFailure( memberName );
		return false;
	}
	if( !GetPoseYaml( extrinsics, registration.extrinsics ) )
	{
		ROS_WARN_STREAM( "Could not parse extrinsics information for: " << memberName
		    << " at key: " << extrinsicsKey );
		RecordFailure( memberName );
		return false;
	}
	
	// Then read reference frame ID
	std::string referenceKey = GenerateRefFrameKey( memberNamespace );
	if( !nodeHandle.getParam( referenceKey, registration.referenceFrame ) )
	{
		ROS_WARN_STREAM( "Could not find reference frame information for: " << memberName
		    << " at path " << referenceKey );
		RecordFailure( memberName );
		return false;
	}
	
	RegisterMember( memberName, registration );
	return true;
}

bool ExtrinsicsInfoManager::WriteMemberInfo( const std::string& memberName,
                                             bool forceLookup,
	                                         const ros::Duration& timeout )
{
	// Can't write info if we don't have it cached!
	if( !HasMember( memberName ) ) { return false; }
	
	std::string memberNamespace;
	if( !GetNamespace( memberName, memberNamespace, forceLookup, timeout ) ) 
	{ 
		return false; 
	}
	
	ExtrinsicsInfo& registration = GetInfo( memberName );
	YAML::Node extrinsics = SetPoseYaml( registration.extrinsics );
	SetYamlParam( nodeHandle, GenerateExtrinsicsKey( memberNamespace ), extrinsics );
	nodeHandle.setParam( GenerateRefFrameKey( memberNamespace ), registration.referenceFrame );
	return true;
}

std::string ExtrinsicsInfoManager::GenerateExtrinsicsKey( const std::string& ns )
{
	return ns + "extrinsics";
}

std::string ExtrinsicsInfoManager::GenerateRefFrameKey( const std::string& ns )
{
	return ns + "frame_id";
}

} // end namespace extrinsics_array
