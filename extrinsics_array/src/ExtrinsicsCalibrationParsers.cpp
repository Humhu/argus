#include "argus_utils/utils/YamlUtils.h"
#include "argus_utils/geometry/GeometryUtils.h"

#include "extrinsics_array/ExtrinsicsCalibrationParsers.h"

#include <fstream>

namespace argus
{

bool ParseExtrinsicsCalibration( const YAML::Node& yaml,
                                 ExtrinsicsInfo& info )
{
	if( !yaml["frame_id"] || !yaml["extrinsics"] ) 
	{ 
		std::cerr << "Could not find all required fields in config file." << std::endl;
		return false; 
	}
	
	info.referenceFrame = yaml["frame_id"].as<std::string>();
	
	PoseSE3 pose;
	if( !GetPoseYaml( yaml["extrinsics"], pose ) )
	{
		return false;
	}
	info.extrinsics = pose;

	return true;
}
	
bool ReadExtrinsicsCalibration( const std::string& path, 
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
	return ParseExtrinsicsCalibration( yaml, info );
}
	
bool PopulateExtrinsicsCalibration( const ExtrinsicsInfo& info,
                                    YAML::Node& yaml )
{
	yaml["frame_id"] = info.referenceFrame;
	yaml["extrinsics"] = SetPoseYaml( info.extrinsics );
	return true;
}
	
bool WriteExtrinsicsCalibration( const std::string& path,
                                 const ExtrinsicsInfo& info )
{
	std::ofstream output( path );
	if( !output.is_open() )
	{
		return false;
	}
	
	YAML::Node yaml;
	if( !PopulateExtrinsicsCalibration( info, yaml ) ) { return false; }
	output << yaml;
	return true;
}

} // end namespace extrinsics_array
