#include "fiducials/FiducialInfoManager.h"
#include "fiducials/FiducialCalibrationParsers.h"

#include "argus_utils/YamlUtils.h"

#include <boost/foreach.hpp>

using namespace argus_utils;

namespace fiducials
{

FiducialInfoManager::FiducialInfoManager( lookup::LookupInterface& interface )
: lookupInterface( interface )
{}

bool FiducialInfoManager::ReadFiducialInformation( const std::string& fidName,
                                                   bool forceRead )
{
	// Fast-fail using cache
	if( !forceRead && failedQueries.count( fidName ) > 0 ) { return false; }
	
	std::string fidNamespace;
	if( !lookupInterface.ReadNamespace( fidName, fidNamespace ) )
	{
		ROS_WARN_STREAM( "Could not find namespace for: " << fidName );
		failedQueries.insert( fidName );
		return false;
	}
	
	YAML::Node intrinsics;
	std::string intrinsicsKey = fidNamespace + "intrinsics";
	if( !GetYamlParam( nodeHandle, intrinsicsKey, intrinsics ))
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
	fiducials[fidName] = Fiducial( info );
	return true;
}

bool FiducialInfoManager::HasFiducial( const std::string& fidName ) const
{
	return fiducials.count( fidName ) > 0;
}

const Fiducial& FiducialInfoManager::GetIntrinsics( const std::string& fidName )
{
	return fiducials.at( fidName );
}

} // end namespace fiducials
