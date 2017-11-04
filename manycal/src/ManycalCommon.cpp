#include "manycal/ManycalCommon.h"

#define STATIC_STR ("static")
#define DYN_STR ("dynamic")
#define DISC_STR ("discontinuous")

namespace argus
{
TargetType string_to_target( const std::string& str )
{
	if( str == STATIC_STR )
	{
		return TargetType::TARGET_STATIC;
	}
	else if( str == DYN_STR )
	{
		return TargetType::TARGET_DYNAMIC;
	}
	else if( str == DISC_STR )
	{
		return TargetType:: TARGET_DISCONTINUOUS;
	}
	else
	{
		throw std::invalid_argument( "Unknown target type: " + str );
	}
}

std::string target_to_string( TargetType type )
{
	if( type == TargetType::TARGET_STATIC ) { return STATIC_STR; }
	else if( type == TargetType::TARGET_DYNAMIC ) { return DYN_STR;
	}
	else if( type == TargetType::TARGET_DISCONTINUOUS ) { return DISC_STR;
	}
	else { throw std::invalid_argument( "Unrecognized target type" ); }
}

isam::PoseSE3 PoseToIsam( const argus::PoseSE3& pose )
{
	return isam::PoseSE3( pose );
}

argus::PoseSE3 IsamToPose( const isam::PoseSE3& is )
{
	return is.pose;
}

isam::Point3d PointToIsam( const Translation3Type& msg )
{
	return isam::Point3d( msg.x(), msg.y(), msg.z() );
}

Translation3Type IsamToPoint( const isam::Point3d& is )
{
	return Translation3Type( is.x(), is.y(), is.z() );
}

isam::FiducialDetection DetectionToIsam( const FiducialDetection& detection )
{
	Eigen::VectorXd detectionVector = Eigen::VectorXd( 2 * detection.points.size() );
	for( unsigned int i = 0; i < detection.points.size(); i++ )
	{
		detectionVector.block<2, 1>( 2 * i, 0 ) = Eigen::Vector2d( detection.points[i].x(),
		                                                           detection.points[i].y() );
	}
	return isam::FiducialDetection( detectionVector );
}

FiducialDetection IsamToDetection( const isam::FiducialDetection& detection )
{
	Eigen::VectorXd allPoints = detection.vector();
	FiducialDetection out;
	unsigned int numPoints = allPoints.rows() / 2;
	out.points.reserve( numPoints );
	for( unsigned int i = 0; i < numPoints; i++ )
	{
		out.points.emplace_back( allPoints( 2 * i ), allPoints( 2 * i + 1 ) );
	}
	return out;
}

isam::FiducialIntrinsics FiducialToIsam( const Fiducial& fid )
{
	std::vector<isam::Point3d> pts;
	pts.reserve( fid.points.size() );
	for( unsigned int i = 0; i < fid.points.size(); i++ )
	{
		pts.push_back( PointToIsam( fid.points[i] ) );
	}

	return isam::FiducialIntrinsics( pts );
}

Fiducial IsamToFiducial( const isam::FiducialIntrinsics& fid )
{
	isam::FiducialIntrinsics::MatrixType fidMat = fid.matrix();

	Fiducial output;
	output.points.reserve( fidMat.cols() );
	for( unsigned int i = 0; i < fidMat.cols(); i++ )
	{
		output.points.emplace_back( fidMat( 0, i ), fidMat( 1, i ), fidMat( 2, i ) );
	}
	return output;
}

isam::MonocularIntrinsics CalibrationToIsam( const CameraCalibration& calib )
{
	return isam::MonocularIntrinsics( calib.GetFx(), calib.GetFy(),
	                                  calib.GetCx(), calib.GetCy() );
}

CameraCalibration IsamToCalibration( const isam::MonocularIntrinsics& calib )
{
	std::cerr << "Warning: IsamToCalibration is not implemented" << std::endl;
	return CameraCalibration();
}
}
