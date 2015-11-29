#include "fiducials/FiducialInfoManager.h"
#include "fiducials/FiducialCalibrationParsers.h"

#include "argus_utils/ParamUtils.h"
#include "argus_utils/YamlUtils.h"

#include <boost/foreach.hpp>

using namespace argus_utils;

namespace fiducials
{

FiducialInfoManager::FiducialInfoManager( lookup::LookupInterface& interface )
: lookupInterface( interface )
{}

bool FiducialInfoManager::ReadFiducialInformation( const std::string& fidName,
                                                   bool forceLookup )
{
	// Retrieve intrinsics YAML
	YAML::Node intrinsics;
	std::string fidNamespace;
	if( !GetNamespace( fidName, fidNamespace, forceLookup ) ) { return false; }
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
	fiducials[fidName] = Fiducial( info );
	return true;
}

bool FiducialInfoManager::WriteFiducialInformation( const std::string& fidName,
                                                    bool forceLookup )
{
	// Can't write params if we don't have them cached!
	if( fiducials.count( fidName ) == 0 ) { return false; }
	
	YAML::Node yaml;
	PopulateFiducialCalibration( fiducials[ fidName ].ToInfo(), yaml );
	std::string fidNamespace;
	if( !GetNamespace( fidName, fidNamespace, forceLookup ) ) { return false; }
	std::string intrinsicsKey = GenerateIntrinsicsKey( fidNamespace );
	SetYamlParam( nodeHandle, intrinsicsKey, yaml );
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

void FiducialInfoManager::SetIntrinsics( const std::string& fidName, const Fiducial& fid )
{
	fiducials[ fidName ] = fid;
}

bool FiducialInfoManager::GetNamespace( const std::string& fidName, 
                                        std::string& ns,
                                        bool forceLookup )
{
	// Fast-fail using cache
	if( !forceLookup && failedQueries.count( fidName ) > 0 ) { return false; }
	
	if( !lookupInterface.ReadNamespace( fidName, ns ) )
	{
		ROS_WARN_STREAM( "Could not find namespace for: " << fidName );
		failedQueries.insert( fidName );
		return false;
	}
	return true;
}

std::string FiducialInfoManager::GenerateIntrinsicsKey( const std::string& ns )
{
	return ns + "intrinsics";
}

} // end namespace fiducials
