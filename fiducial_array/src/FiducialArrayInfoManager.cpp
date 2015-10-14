#include "fiducial_array/FiducialArrayInfoManager.h"
#include "fiducial_array/FiducialArrayCalibrationParsers.h"

#include "argus_utils/YamlUtils.h"

#include <boost/foreach.hpp>

using namespace argus_utils;

namespace fiducial_array
{
	
FiducialArrayInfoManager::FiducialArrayInfoManager( ros::NodeHandle& nh )
: nodeHandle( nh )
{}

void FiducialArrayInfoManager::ReadFiducialInformation( const std::string& paramRoot )
{
	YAML::Node fiducials;
	if( !GetYamlParam( nodeHandle, paramRoot, fiducials ) )
	{
		ROS_ERROR_STREAM( "Error reading fiducial array information at root " << paramRoot );
		exit( -1 );
	}
	
	YAML::Node::const_iterator iter;
	std::string refName;
	for( iter = fiducials.begin(); iter != fiducials.end(); iter++ )
	{
		std::string arrayName = iter->first.as<std::string>();
		const YAML::Node& info = iter->second;
		
		// Parse the array info from the YAML
		std::shared_ptr<FiducialArrayInfo> arrayInfo = std::make_shared<FiducialArrayInfo>();
		if( !ParseFiducialArrayCalibration( info, refName, *arrayInfo ) )
		{
			ROS_ERROR_STREAM( "Error parsing fiducial information for " << arrayName );
			continue;
		}
		
		// Add the info to the array name map
		if( arrayMap.count( arrayName ) != 0 )
		{
			ROS_ERROR_STREAM( "Fiducial array " << arrayName << " already exists!" );
			continue;
		}
		arrayMap[ arrayName ] = arrayInfo;
		
		// Add the info to the fiducial name map
		BOOST_FOREACH( const std::string& fidName, arrayInfo->fiducialNames )
		{
			if( fidMap.count( fidName ) != 0 )
			{
				ROS_ERROR_STREAM( "Fiducial " << fidName << " already exists!" );
				continue;
			}
			fidMap[ fidName ] = arrayInfo;
		}
	}
}

const FiducialArrayInfo& FiducialArrayInfoManager::LookupArray( const std::string& arrayName ) const
{
	return *(arrayMap.at( arrayName ));
}

const FiducialArrayInfo& FiducialArrayInfoManager::LookupFiducial( const std::string& fidName ) const
{
	return *(fidMap.at( fidName ));
}
	
} // end namespace fiducial_array
