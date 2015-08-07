#include "extrinsics_info_manager/ExtrinsicsInfoManager.h"
#include "extrinsics_info_manager/ExtrinsicsCalibrationParsers.h"

namespace extrinsics_info_manager
{
	
	ExtrinsicsInfoManager::ExtrinsicsInfoManager( ros::NodeHandle& nh,
												  ros::NodeHandle& ph )
	: nodeHandle( nh ), privHandle( ph ), referenceName( "reference" ), initialized( false )
	{
		getInfoServer = privHandle.advertiseService( "get_extrinsics_info",
							&ExtrinsicsInfoManager::GetExtrinsicsInfoService, this );
		setInfoServer = privHandle.advertiseService( "set_extrinsics_info",
							&ExtrinsicsInfoManager::SetExtrinsicsInfoService, this );
	}
	
	void ExtrinsicsInfoManager::SetReferenceName( const std::string& name )
	{
		referenceName = name;
	}
	
	bool ExtrinsicsInfoManager::LoadExtrinsicsInfo( const std::string& filePath )
	{
		 initialized = ReadExtrinsicsCalibration( filePath, referenceName, extrinsics );
		 return initialized;
	}
	
	bool ExtrinsicsInfoManager::IsCalibrated() const
	{
		return initialized;
	}
	
	ExtrinsicsInfo ExtrinsicsInfoManager::GetExtrinsicsInfo() const
	{
		return extrinsics;
	}
	
	void ExtrinsicsInfoManager::SetExtrinsicsInfo( const ExtrinsicsInfo& info )
	{
		extrinsics = info;
	}
	
	bool ExtrinsicsInfoManager::GetExtrinsicsInfoService( GetExtrinsicsInfo::Request& req,
														  GetExtrinsicsInfo::Response& res )
	{
		if( !initialized ) { return false; }
		res.info = extrinsics;
		return true;
	}
	
	bool ExtrinsicsInfoManager::SetExtrinsicsInfoService( SetExtrinsicsInfo::Request& req,
														  SetExtrinsicsInfo::Response& res )
	{
		extrinsics = req.info;
		initialized = true;
		return true;
	}
	
}
