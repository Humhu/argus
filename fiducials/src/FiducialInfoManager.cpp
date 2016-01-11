#include "fiducials/FiducialInfoManager.h"
#include "fiducials/FiducialCalibrationParsers.h"

#include "argus_utils/ParamUtils.h"
#include "argus_utils/YamlUtils.h"

#include <boost/foreach.hpp>

using namespace argus_utils;

namespace fiducials
{

FiducialInfoManager::FiducialInfoManager( lookup::LookupInterface& interface )
: InfoManager( interface )
{}

bool FiducialInfoManager::ReadMemberInfo( const std::string& fidName,
                                          bool forceLookup,
                                          const ros::Duration& timeout )
{
	// Retrieve intrinsics YAML
	YAML::Node intrinsics;
	std::string fidNamespace;
	if( !GetNamespace( fidName, fidNamespace, forceLookup, timeout ) ) 
	{ 
		return false; 
	}
	std::string intrinsicsKey = GenerateIntrinsicsKey( fidNamespace );
	
	if( !GetYamlParam( nodeHandle, intrinsicsKey, intrinsics ) )
	{
		ROS_WARN_STREAM( "Could not find intrinsics information for: " << fidName
		    << " at path: " << intrinsicsKey );
		return false;
	}
	FiducialInfo info;
	if( !ParseFiducialCalibration( intrinsics, info ) )
	{
		ROS_WARN_STREAM( "Could not parse intrinsics information for: " << fidName
		    << " at path: " << intrinsicsKey );
		return false;
	}
	RegisterMember( fidName, Fiducial( info ) );
	return true;
}

bool FiducialInfoManager::WriteMemberInfo( const std::string& fidName,
                                           bool forceLookup,
                                           const ros::Duration& timeout )
{
	// Can't write params if we don't have them cached!
	if( !HasMember( fidName ) ) { return false; }
	
	YAML::Node yaml;
	PopulateFiducialCalibration( GetInfo( fidName ).ToInfo(), yaml );
	std::string fidNamespace;
	if( !GetNamespace( fidName, fidNamespace, forceLookup, timeout ) ) 
	{ 
		return false; 
	}
	std::string intrinsicsKey = GenerateIntrinsicsKey( fidNamespace );
	SetYamlParam( nodeHandle, intrinsicsKey, yaml );
	return true;
}

std::string FiducialInfoManager::GenerateIntrinsicsKey( const std::string& ns )
{
	return ns + "intrinsics";
}

} // end namespace fiducials
