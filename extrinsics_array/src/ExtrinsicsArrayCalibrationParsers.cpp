#include "argus_utils/YamlUtils.h"
#include "argus_utils/GeometryUtils.h"

#include "extrinsics_array/ExtrinsicsArrayCalibrationParsers.h"

#include <fstream>

using namespace argus_utils;

namespace extrinsics_array
{

bool ParseExtrinsicsArrayCalibration( const YAML::Node& yaml,
                                      ExtrinsicsArrayInfo& info )
{
	if( !yaml["frame_id"] ) 
	{ 
		std::cerr << "Could not find all required fields in config file." << std::endl;
		return false; 
	}
	
	// Prepare info for writing
	info.memberNames.clear();
	info.extrinsics.clear();
	
	YAML::Node::const_iterator iter;
	for( iter = yaml.begin(); iter != yaml.end(); iter++ )
	{
		std::string memberName = iter->first.as<std::string>();
		
		// Catch for frame_id case (can't be a member name)
		if( memberName == "frame_id" ) 
		{ 
			info.frame_id = iter->second.as<std::string>();
			continue;
		}
		
		PoseSE3 pose;
		// If it doesn't have extrinsics, just skip it
		if( !GetPoseYaml( iter->second["extrinsics"], pose ) ) 
		{ 
			continue; 
		}
		geometry_msgs::Pose extrinsic = PoseToMsg( pose );
		
		info.memberNames.push_back( memberName );
		info.extrinsics.push_back( extrinsic );
	}
	return true;
}
	
bool ReadExtrinsicsArrayCalibration( const std::string& path, 
                                     ExtrinsicsArrayInfo& info )
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
	return ParseExtrinsicsArrayCalibration( yaml, info );
}
	
bool PopulateExtrinsicsArrayCalibration( const ExtrinsicsArrayInfo& info,
                                         YAML::Node& yaml )
{
	if( (info.memberNames.size() != info.extrinsics.size()) ) { 
		std::cerr << "Malformed extrinsics info message." << std::endl;
		return false; 
	}
	
	for( unsigned int i = 0; i < info.memberNames.size(); i++ )
	{
		std::string memberName = info.memberNames[i];
		PoseSE3 extrinsic = MsgToPose( info.extrinsics[i] );
		
		YAML::Node fidNode;
		
		fidNode["extrinsics"] = SetPoseYaml( extrinsic );
		
		yaml[memberName] = fidNode;
	}
	
	yaml[ "frame_id" ] = info.frame_id;
	return true;
}
	
bool WriteExtrinsicsArrayCalibration( const std::string& path,
                                      const ExtrinsicsArrayInfo& info )
{
	std::ofstream output( path );
	if( !output.is_open() )
	{
		return false;
	}
	
	YAML::Node yaml;
	if( !PopulateExtrinsicsArrayCalibration( info, yaml ) ) { return false; }
	output << yaml;
	return true;
}

} // end namespace extrinsics_array
