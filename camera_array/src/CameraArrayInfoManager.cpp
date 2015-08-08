#include "camera_array/CameraArrayInfoManager.h"
#include "camera_array/CameraArrayCalibrationParsers.h"

namespace camera_array
{
	
	CameraArrayInfoManager::CameraArrayInfoManager( ros::NodeHandle& nh,
												  ros::NodeHandle& ph )
	: nodeHandle( nh ), privHandle( ph ), referenceName( "reference" ), initialized( false )
	{
		getInfoServer = privHandle.advertiseService( "get_extrinsics_info",
							&CameraArrayInfoManager::GetCameraArrayInfoService, this );
		setInfoServer = privHandle.advertiseService( "set_extrinsics_info",
							&CameraArrayInfoManager::SetCameraArrayInfoService, this );
	}
	
	void CameraArrayInfoManager::SetReferenceName( const std::string& name )
	{
		referenceName = name;
	}
	
	bool CameraArrayInfoManager::LoadCameraArrayInfo( const std::string& filePath )
	{
		 initialized = ReadCameraArrayCalibration( filePath, referenceName, extrinsics );
		 return initialized;
	}
	
	bool CameraArrayInfoManager::IsCalibrated() const
	{
		return initialized;
	}
	
	CameraArrayInfo CameraArrayInfoManager::GetCameraArrayInfo() const
	{
		return extrinsics;
	}
	
	void CameraArrayInfoManager::SetCameraArrayInfo( const CameraArrayInfo& info )
	{
		extrinsics = info;
	}
	
	bool CameraArrayInfoManager::GetCameraArrayInfoService( GetCameraArrayInfo::Request& req,
														  GetCameraArrayInfo::Response& res )
	{
		if( !initialized ) { return false; }
		res.info = extrinsics;
		return true;
	}
	
	bool CameraArrayInfoManager::SetCameraArrayInfoService( SetCameraArrayInfo::Request& req,
														  SetCameraArrayInfo::Response& res )
	{
		extrinsics = req.info;
		initialized = true;
		return true;
	}
	
}
