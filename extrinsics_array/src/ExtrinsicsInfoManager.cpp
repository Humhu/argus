#include "extrinsics_array/ExtrinsicsInfoManager.h"

#include "argus_utils/utils/YamlUtils.h"
#include "argus_utils/utils/ParamUtils.h"

#include <boost/foreach.hpp>

namespace argus
{
	
ExtrinsicsInfoManager::ExtrinsicsInfoManager( LookupInterface& interface )
: InfoManager( interface )
{}

bool ExtrinsicsInfoManager::ParseMemberInfo( const std::string& memberNamespace,
                                             ExtrinsicsInfo& info )
{
	// Read extrinsics first
	YAML::Node extrinsics;
	std::string extrinsicsKey = GenerateExtrinsicsKey( memberNamespace );
	if( !GetParam( _nodeHandle, extrinsicsKey, extrinsics ) )
	{
		ROS_WARN_STREAM( "Could not find extrinsics information at path " << extrinsicsKey );
		return false;
	}
	if( !GetPoseYaml( extrinsics, info.extrinsics ) )
	{
		ROS_WARN_STREAM( "Could not parse extrinsics information at key: " << extrinsicsKey );
		return false;
	}
	
	// Then read reference frame ID
	std::string referenceKey = GenerateRefFrameKey( memberNamespace );
	if( !_nodeHandle.getParam( referenceKey, info.referenceFrame ) )
	{
		ROS_WARN_STREAM( "Could not find reference frame information at path " << referenceKey );
		return false;
	}
	
	return true;
}

void ExtrinsicsInfoManager::PopulateMemberInfo( const ExtrinsicsInfo& info,
                                                const std::string& memberNamespace )
{
	YAML::Node extrinsics = SetPoseYaml( info.extrinsics );
	SetYamlParam( _nodeHandle, GenerateExtrinsicsKey( memberNamespace ), extrinsics );
	_nodeHandle.setParam( GenerateRefFrameKey( memberNamespace ), info.referenceFrame );
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
