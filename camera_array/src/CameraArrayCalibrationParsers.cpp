#include "argus_utils/YamlUtils.h"
#include "argus_utils/GeometryUtils.h"

#include "camera_array/CameraArrayCalibrationParsers.h"

#include "camplex/CameraCalibration.h"

#include <fstream>

using namespace argus_utils;
using namespace camera_calibration_parsers;

namespace camera_array
{

bool ReadCameraArrayCalibration( const std::string& path,
								std::string& refName,
								CameraArrayInfo& info )
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
	
	if( !yaml["array_name"] || !yaml["cameras"] ) { return false; }
	
	// Prepare info for writing
	info.cameraNames.clear();
	info.intrinsics.clear();
	info.extrinsics.clear();
	
	refName = yaml["array_name"].as<std::string>();
	
	YAML::Node cameras = yaml["cameras"];
	YAML::Node::const_iterator iter;
	for( iter = cameras.begin(); iter != cameras.end(); iter++ )
	{
		std::string cameraName = iter->first.as<std::string>();
		
		PoseSE3 pose;
		if( !GetPoseYaml( iter->second["extrinsics"], pose ) ) { return false; }
		geometry_msgs::Pose extrinsic = PoseToMsg( pose );
		
		std::vector<double> focal = iter->second["intrinsics"]["focal_length"].as< std::vector<double> >();
		std::vector<double> principal= iter->second["intrinsics"]["principal_point"].as< std::vector<double> >();
		std::vector<double> resolution = iter->second["intrinsics"]["resolution"].as< std::vector<double> >();
		camplex::CameraCalibration cc( cameraName, focal[0], focal[1], 
									   principal[0], principal[1],
									   resolution[0], resolution[1] );
		sensor_msgs::CameraInfo intrinsic = cc.GetInfo();
		
		info.cameraNames.push_back( cameraName );
		info.extrinsics.push_back( extrinsic );
		info.intrinsics.push_back( intrinsic );
	}
	
	return true;
}
	
bool WriteCameraArrayCalibration( const std::string& path,
								 const std::string& refName,
								 const CameraArrayInfo& info )
{
	std::ofstream output( path );
	if( !output.is_open() )
	{
		return false;
	}
	
	YAML::Node yaml;
	yaml["array_name"] = refName;
	
	YAML::Node cameras;
	if( info.cameraNames.size() != info.extrinsics.size() ) { return false; }
	for( unsigned int i = 0; i < info.cameraNames.size(); i++ )
	{
		std::string cameraName = info.cameraNames[i];
		PoseSE3 extrinsic = MsgToPose( info.extrinsics[i] );
		
		YAML::Node camNode;
		
		camNode["extrinsics"] = SetPoseYaml( extrinsic );
		
		camplex::CameraCalibration cc( cameraName, info.intrinsics[i] );
		std::vector<double> focal(2), principal(2), resolution(2); // = { cc.GetFx(), cc.GetFy() };
		focal[0] = cc.GetFx(); focal[1] = cc.GetFy();
		principal[0] = cc.GetCx(); principal[1] = cc.GetCy();
		cv::Size scale = cc.GetScale();
		resolution[0] = scale.width; resolution[1] = scale.height;
		
		camNode["intrinsics"]["focal_length"] = focal;
		camNode["intrinsics"]["principal_point"] = principal;
		camNode["intrinsics"]["resolution"] = resolution;
		
		cameras[cameraName] = camNode;
	}
	
	yaml["cameras"] = cameras;
	output << yaml;
	return true;
}

} // end namespace camera_array
