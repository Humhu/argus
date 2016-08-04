#include "fiducials/FiducialInfoManager.h"
#include "fiducials/FiducialCalibrationParsers.h"

#include "argus_utils/utils/ParamUtils.h"
#include "argus_utils/utils/YamlUtils.h"

#include <boost/foreach.hpp>

namespace argus
{

FiducialInfoManager::FiducialInfoManager( LookupInterface& interface )
: InfoManager( interface ) {}

bool FiducialInfoManager::ParseMemberInfo( const std::string& memberNamespace,
                                           Fiducial& fid )
{
	YAML::Node intrinsics;
	std::string intrinsicsKey = GenerateIntrinsicsKey( memberNamespace );
	if( !GetParam( _nodeHandle, intrinsicsKey, intrinsics ) )
	{
		ROS_WARN_STREAM( "Could not find intrinsics information at path: " << intrinsicsKey );
		return false;
	}
	fiducials::FiducialInfo info;
	if( !ParseFiducialCalibration( intrinsics, info ) )
	{
		ROS_WARN_STREAM( "Could not parse intrinsics information at path: " << intrinsicsKey );
		return false;
	}
	fid = Fiducial( info );
	return true;
}

void FiducialInfoManager::PopulateMemberInfo( const Fiducial& fid,
                                              const std::string& memberNamespace )
{
	YAML::Node yaml;
	PopulateFiducialCalibration( fid.ToInfo(), yaml );
	std::string intrinsicsKey = GenerateIntrinsicsKey( memberNamespace );
	SetYamlParam( _nodeHandle, intrinsicsKey, yaml );
}

std::string FiducialInfoManager::GenerateIntrinsicsKey( const std::string& ns )
{
	return ns + "intrinsics";
}

} // end namespace fiducials
