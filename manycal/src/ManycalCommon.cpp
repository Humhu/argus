#include "manycal/ManycalCommon.h"

namespace argus
{
	
isam::Pose3d PoseToIsam( const argus::PoseSE3& pose )
{
	return isam::Pose3d( pose.ToTransform() );
}

argus::PoseSE3 IsamToPose( const isam::Pose3d& is )
{
	return argus::PoseSE3( is.wTo() );
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
	Eigen::VectorXd detectionVector = Eigen::VectorXd( 2*detection.points.size() );
	for( unsigned int i = 0; i < detection.points.size(); i++ )
	{
		detectionVector.block<2,1>(2*i,0) = Eigen::Vector2d( detection.points[i].x(), 
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
		out.points.emplace_back( allPoints( 2*i ), allPoints( 2*i + 1 ) );
	}
	return out;
}

isam::FiducialIntrinsics FiducialToIsam( const Fiducial& fid )
{
	std::vector <isam::Point3d> pts;
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
		output.points.emplace_back( fidMat(0,i), fidMat(1,i), fidMat(2,i) );
	}
	return output;
}

}
