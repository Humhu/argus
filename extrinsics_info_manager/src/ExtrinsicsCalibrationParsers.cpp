#include "argus_utils/YamlUtils.h"
#include "argus_utils/GeometryUtils.h"

#include "extrinsics_info_manager/ExtrinsicsCalibrationParsers.h"

#include <fstream>

using namespace argus_utils;

namespace extrinsics_info_manager
{

bool ReadExtrinsicsCalibration( const std::string& path,
								std::string& refName,
								ExtrinsicsInfo& info )
{
	YAML::Node yaml;
	try 
	{
		 yaml = YAML::LoadFile( path );
	}
	catch( YAML::BadFile e )
	{
		return false;
	}
	
	if( !yaml["reference_name"] || !yaml["frames"] ) { return false; }
	
	// Prepare info for writing
	info.frameNames.clear();
	info.extrinsics.clear();
	
	refName = yaml["reference_name"].as<std::string>();
	
	YAML::Node frames = yaml["frames"];
	YAML::Node::const_iterator iter;
	for( iter = frames.begin(); iter != frames.end(); iter++ )
	{
		std::string frameName = iter->first.as<std::string>();
		PoseSE3 extrinsic;
		if( !GetPoseYaml( iter->second, extrinsic ) ) { return false; }
		geometry_msgs::Pose pose = PoseToMsg( extrinsic );
		
		info.frameNames.push_back( frameName );
		info.extrinsics.push_back( pose );
	}
	
	return true;
}
	
bool WriteExtrinsicsCalibration( const std::string& path,
								 const std::string& refName,
								 const ExtrinsicsInfo& info )
{
	std::ofstream output( path );
	if( !output.is_open() )
	{
		return false;
	}
	
	YAML::Node yaml;
	yaml["reference_name"] = refName;
	
	YAML::Node frames;
	if( info.frameNames.size() != info.extrinsics.size() ) { return false; }
	for( unsigned int i = 0; i < info.frameNames.size(); i++ )
	{
		std::string frameName = info.frameNames[i];
		PoseSE3 extrinsic = MsgToPose( info.extrinsics[i] );
		frames[frameName] = SetPoseYaml( extrinsic );
	}
	
	yaml["frames"] = frames;
	output << yaml;
	return true;
}

} // end namespace extrinsics_info_manager
