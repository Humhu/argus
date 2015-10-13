#include "fiducial_array/FiducialArrayInfoManager.h"
#include "fiducial_array/FiducialArrayCalibrationParsers.h"

namespace fiducial_array
{
	
FiducialArrayInfoManager::FiducialArrayInfoManager( ros::NodeHandle& nh, ros::NodeHandle& ph )
: nodeHandle( nh ), privHandle( ph ), referenceName( "reference" ), initialized( false )
{
	getInfoServer = nodeHandle.advertiseService( "get_fiducial_array_info",
	                    &FiducialArrayInfoManager::GetFiducialArrayInfoService, this );
	setInfoServer = nodeHandle.advertiseService( "set_fiducial_array_info",
	                    &FiducialArrayInfoManager::SetFiducialArrayInfoService, this );
}

void FiducialArrayInfoManager::SetReferenceName( const std::string& name )
{
	referenceName = name;
}

bool FiducialArrayInfoManager::LoadFiducialArrayInfo( const std::string& filePath )
{
	 initialized = ReadFiducialArrayCalibration( filePath, referenceName, extrinsics );
	 return initialized;
}

bool FiducialArrayInfoManager::IsCalibrated() const
{
	return initialized;
}

FiducialArrayInfo FiducialArrayInfoManager::GetFiducialArrayInfo() const
{
	return extrinsics;
}

void FiducialArrayInfoManager::SetFiducialArrayInfo( const FiducialArrayInfo& info )
{
	extrinsics = info;
}

bool FiducialArrayInfoManager::GetFiducialArrayInfoService( GetFiducialArrayInfo::Request& req,
                                                            GetFiducialArrayInfo::Response& res )
{
	if( !initialized ) { return false; }
	res.info = extrinsics;
	return true;
}

bool FiducialArrayInfoManager::SetFiducialArrayInfoService( SetFiducialArrayInfo::Request& req,
                                                            SetFiducialArrayInfo::Response& res )
{
	extrinsics = req.info;
	initialized = true;
	return true;
}
	
} // end namespace fiducial_array
