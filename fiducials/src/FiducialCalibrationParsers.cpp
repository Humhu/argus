#include "argus_utils/utils/YamlUtils.h"
#include "argus_utils/geometry/GeometryUtils.h"
#include "fiducials/FiducialCalibrationParsers.h"

#include <fstream>
#include <boost/foreach.hpp>

namespace argus
{

bool ParseFiducialCalibration( const YAML::Node& yaml, fiducials::FiducialInfo& info )
{
	// Only the point are required
	if( !yaml["points_x"] ||
		!yaml["points_y"] ||
		!yaml["points_z"] )
	{
		std::cerr << "Could not read fiducial info from YAML." << std::endl;
		return false;
	}
	
	// Parse the points
	std::vector<double> pointsX = yaml["points_x"].as< std::vector<double> >();
	std::vector<double> pointsY = yaml["points_y"].as< std::vector<double> >();
	std::vector<double> pointsZ = yaml["points_z"].as< std::vector<double> >();
	if( (pointsX.size() != pointsY.size()) && (pointsY.size() != pointsZ.size()) )
	{
		std::cerr << "Point fields must have same number of elements." << std::endl;
		return false;
	}

	info.points.clear();
	info.points.reserve( pointsX.size() );
	geometry_msgs::Point point;
	for( unsigned int i = 0; i < pointsX.size(); i++ )
	{
		point.x = pointsX[i];
		point.y = pointsY[i];
		point.z = pointsZ[i];
		info.points.push_back( point );
	}
	return true;
}

bool ReadFiducialCalibration( const std::string& path, fiducials::FiducialInfo& info )
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
	
	return ParseFiducialCalibration( yaml, info );
}

void PopulateFiducialCalibration( const fiducials::FiducialInfo& info, YAML::Node& yaml )
{
	std::vector<double> pointsX, pointsY, pointsZ;
	for( unsigned int i = 0; i < info.points.size(); i++ )
	{
		pointsX.push_back( info.points[i].x );
		pointsY.push_back( info.points[i].y );
		pointsZ.push_back( info.points[i].z );
	}
	yaml[ "intrinsics" ][ "points_x" ] = pointsX;
	yaml[ "intrinsics" ][ "points_y" ] = pointsY;
	yaml[ "intrinsics" ][ "points_z" ] = pointsZ;
}

bool WriteFiducialCalibration( const std::string& path, const fiducials::FiducialInfo& info )
{
	std::ofstream output( path );
	if( !output.is_open() )
	{
		return false;
	}
	
	YAML::Node yaml;
	PopulateFiducialCalibration( info, yaml );
	output << yaml;
	return true;
}

} // end namespace fiducials
