#include "fiducial_array/FiducialInfoManager.h"
#include "fiducial_array/FiducialCalibrationParsers.h"

#include "argus_utils/YamlUtils.h"

#include <boost/foreach.hpp>

using namespace argus_utils;
using namespace extrinsics_array;

namespace fiducial_array
{
	
FiducialInfoManager::FiducialInfoManager( ros::NodeHandle& nh )
: ExtrinsicsInfoManager( nh )
{}

FiducialInfoManager::~FiducialInfoManager() {}

bool FiducialInfoManager::ReadArrayInformation( const std::string& arrayPath, bool forceRead )
{
	if( !forceRead && failedArrayQueries.count( arrayPath ) > 0 ) { return false; }
	
	YAML::Node fiducials;
	
	std::string p = SanitizeArrayPath( arrayPath );
	if( !GetYamlParam( nodeHandle, p, fiducials ) )
	{
		ROS_WARN_STREAM( "Could not retrieve fiducial array information at " << p );
		failedArrayQueries.insert( arrayPath );
		return false;
	}
	
	// Parse the array info into the map
	FiducialArrayInfo arrayInfo;
	if( !ParseFiducialArrayCalibration( fiducials, arrayInfo ) )
	{
		ROS_ERROR_STREAM( "Error parsing fiducial information for " << p );
		failedArrayQueries.insert( arrayPath );
		return false;
	}
	arrayMap[ p ] = std::make_shared<FiducialArray>( arrayInfo );
	
	// Add the info to the fiducials name map
	BOOST_FOREACH( const std::string& name, arrayInfo.extrinsics.memberNames )
	{
		memberMap[ name ] = p;
	}
	return true;
}

const FiducialArray& FiducialInfoManager::GetFiducialArray( const std::string& arrayPath )
{
	ExtrinsicsArray::Ptr array = arrayMap.at( arrayPath );
	return *( std::dynamic_pointer_cast< FiducialArray >( array ) );
}

const FiducialArray& FiducialInfoManager::GetParentFiducialArray( const std::string& memberName )
{
	ExtrinsicsArray::Ptr array = arrayMap.at( memberMap.at( memberName ) );
	return *( std::dynamic_pointer_cast< FiducialArray >( array ) );
}
	
} // end namespace fiducial_array
