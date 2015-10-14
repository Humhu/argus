#include "argus_utils/YamlUtils.h"
#include "argus_utils/GeometryUtils.h"

#include "fiducial_array/FiducialArrayCalibrationParsers.h"

#include <fstream>

using namespace argus_utils;

namespace fiducial_array
{

bool ParseFiducialArrayCalibration( const YAML::Node& yaml,
                                    std::string& refName,
                                    FiducialArrayInfo& info )
{
	// Prepare info for writing
	info.fiducialNames.clear();
	info.intrinsics.clear();
	info.extrinsics.clear();
	
	refName = yaml["array_name"].as<std::string>();
	
	YAML::Node fiducials = yaml["fiducials"];
	YAML::Node::const_iterator iter;
	for( iter = fiducials.begin(); iter != fiducials.end(); iter++ )
	{
		std::string fiducialName = iter->first.as<std::string>();
		
		PoseSE3 pose;
		if( !GetPoseYaml( iter->second["extrinsics"], pose ) ) 
		{ 
			std::cerr << "Could not read pose." << std::endl;
			return false; 
		}
		geometry_msgs::Pose extrinsic = PoseToMsg( pose );
		
		if( !iter->second["intrinsics"]["points_x"] ||
			!iter->second["intrinsics"]["points_y"] ||
			!iter->second["intrinsics"]["points_z"] ) 
		{ 
			std::cerr << "Could not read intrinsics." << std::endl;
			return false; 
		}
		
		std::vector<double> pointsX = iter->second["intrinsics"]["points_x"].as< std::vector<double> >();
		std::vector<double> pointsY = iter->second["intrinsics"]["points_y"].as< std::vector<double> >();
		std::vector<double> pointsZ = iter->second["intrinsics"]["points_z"].as< std::vector<double> >();
		if( (pointsX.size() != pointsY.size()) && (pointsY.size() != pointsZ.size()) )
		{
			std::cerr << "Point fields must have same number of elements." << std::endl;
			return false;
		}
		
		FiducialInfo intrinsic;
		intrinsic.points.reserve( pointsX.size() );
		geometry_msgs::Point point;
		for( unsigned int i = 0; i < pointsX.size(); i++ )
		{
			point.x = pointsX[i];
			point.y = pointsY[i];
			point.z = pointsZ[i];
			intrinsic.points.push_back( point );
		}
		
		info.fiducialNames.push_back( fiducialName );
		info.intrinsics.push_back( intrinsic );
		info.extrinsics.push_back( extrinsic );
	}
	return true;
}
	
bool ReadFiducialArrayCalibration( const std::string& path, std::string& refName,
                                   FiducialArrayInfo& info )
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
	
	if( !yaml["array_name"] || !yaml["fiducials"] ) 
	{ 
		std::cerr << "Could not find all required fields in config file." << std::endl;
		return false; 
	}
	return ParseFiducialArrayCalibration( yaml, refName, info );
}
	
bool WriteFiducialArrayCalibration( const std::string& path,
                                    const std::string& refName,
                                    const FiducialArrayInfo& info )
{
	std::ofstream output( path );
	if( !output.is_open() )
	{
		return false;
	}
	
	YAML::Node yaml;
	yaml["array_name"] = refName;
	
	YAML::Node fiducials;
	if( (info.fiducialNames.size() != info.intrinsics.size()) ||
		(info.intrinsics.size() != info.extrinsics.size()) ) { 
		std::cerr << "Malformed fiducial info message. Must have same number of elements in arrays." << std::endl;
		return false; 
	}
	
	for( unsigned int i = 0; i < info.fiducialNames.size(); i++ )
	{
		std::string fiducialName = info.fiducialNames[i];
		PoseSE3 extrinsic = MsgToPose( info.extrinsics[i] );
		
		YAML::Node fidNode;
		
		fidNode["extrinsics"] = SetPoseYaml( extrinsic );
		
		std::vector<double> pX( info.intrinsics[i].points.size() );
		std::vector<double> pY( info.intrinsics[i].points.size() );
		std::vector<double> pZ( info.intrinsics[i].points.size() );
		for( unsigned int j = 0; j < info.intrinsics[i].points.size(); j++ )
		{
			pX[j] = info.intrinsics[i].points[j].x;
			pY[j] = info.intrinsics[i].points[j].y;
			pZ[j] = info.intrinsics[i].points[j].z;
		}
		fidNode["intrinsics"]["points_x"] = pX;
		fidNode["intrinsics"]["points_y"] = pX;
		fidNode["intrinsics"]["points_z"] = pX;
		
		fiducials[fiducialName] = fidNode;
	}
	
	yaml["fiducials"] = fiducials;
	output << yaml;
	return true;
}

} // end namespace fiducial_array
