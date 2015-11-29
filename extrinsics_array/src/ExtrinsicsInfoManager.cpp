#include "extrinsics_array/ExtrinsicsInfoManager.h"
#include "extrinsics_array/ExtrinsicsArrayCalibrationParsers.h"

#include "argus_utils/YamlUtils.h"
#include "argus_utils/ParamUtils.h"

#include <boost/foreach.hpp>

using namespace argus_utils;

namespace extrinsics_array
{
	
ExtrinsicsInfoManager::ExtrinsicsInfoManager( lookup::LookupInterface& interface )
: lookupInterface( interface )
{}

bool ExtrinsicsInfoManager::ReadMemberInformation( const std::string& memberName,
                                                   bool forceLookup )
{
	std::string memberNamespace;
	if( !GetNamespace( memberName, memberNamespace, forceLookup ) ) { return false; }

	MemberRegistration registration;
	
	// Read extrinsics first
	YAML::Node extrinsics;
	std::string extrinsicsKey = GenerateExtrinsicsKey( memberNamespace );
	if( !GetYamlParam( nodeHandle, extrinsicsKey, extrinsics ) )
	{
		ROS_WARN_STREAM( "Could not find extrinsics information for: " << memberName 
		    << " at path " << extrinsicsKey );
		failedQueries.insert( memberName );
		return false;
	}
	if( !GetPoseYaml( extrinsics, registration.extrinsics ) )
	{
		ROS_WARN_STREAM( "Could not parse extrinsics information for: " << memberName
		    << " at key: " << extrinsicsKey );
		failedQueries.insert( memberName );
		return false;
	}
	
	// Then read reference frame ID
	std::string referenceKey = GenerateRefFrameKey( memberNamespace );
	if( !nodeHandle.getParam( referenceKey, registration.frameID ) )
	{
		ROS_WARN_STREAM( "Could not find reference frame information for: " << memberName
		    << " at path " << referenceKey );
		failedQueries.insert( memberName );
		return false;
	}
	memberRegistry[ memberName ] = registration;
	return true;
}

bool ExtrinsicsInfoManager::WriteMemberInformation( const std::string& memberName,
                                                    bool forceLookup )
{
	// Can't write info if we don't have it cached!
	if( memberRegistry.count( memberName ) == 0 ) { return false; }
	
	std::string memberNamespace;
	if( !GetNamespace( memberName, memberNamespace, forceLookup ) ) { return false; }
	
	MemberRegistration& registration = memberRegistry[ memberName ];
	YAML::Node extrinsics = SetPoseYaml( registration.extrinsics );
	SetYamlParam( nodeHandle, GenerateExtrinsicsKey( memberNamespace ), extrinsics );
	nodeHandle.setParam( GenerateRefFrameKey( memberNamespace ), registration.frameID );
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

void ExtrinsicsInfoManager::SetExtrinsics( const std::string& memberName, 
                                           const PoseSE3& ext )
{
	memberRegistry[ memberName ].extrinsics = ext;
}

void ExtrinsicsInfoManager::SetReferenceFrame( const std::string& memberName, 
                                               const std::string& refFrame )
{
	memberRegistry[ memberName ].frameID = refFrame;
}

bool ExtrinsicsInfoManager::GetNamespace( const std::string& memberName,
                                          std::string& ns,
                                          bool forceLookup )
{
	// Fast-fail using cache
	if( !forceLookup && failedQueries.count( memberName ) > 0 ) { return false; }
	
	if( !lookupInterface.ReadNamespace( memberName, ns ) )
	{
		ROS_WARN_STREAM( "Could not find namespace for: " << memberName );
		failedQueries.insert( memberName );
		return false;
	}
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
