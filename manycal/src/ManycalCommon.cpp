#include "manycal/ManycalCommon.h"
#include "fiducials/PoseEstimation.h"

using namespace argus;
using namespace argus_msgs;

namespace manycal
{
	
isam::Pose3d PoseToIsam( const argus::PoseSE3& pose )
{
	return isam::Pose3d( pose.ToTransform() );
}

argus::PoseSE3 IsamToPose( const isam::Pose3d& is )
{
	return argus::PoseSE3( is.wTo() );
}

isam::Point3d MsgToIsam( const geometry_msgs::Point& msg )
{
	return isam::Point3d( msg.x, msg.y, msg.z );
}

geometry_msgs::Point IsamToMsg( const isam::Point3d& is )
{
	geometry_msgs::Point msg;
	msg.x = is.x();
	msg.y = is.y();
	msg.z = is.z();
	return msg;
}

isam::FiducialDetection DetectionToIsam( const FiducialDetection& detection )
{
	Eigen::VectorXd detectionVector = Eigen::VectorXd( 2*detection.points.size() );
	for( unsigned int i = 0; i < detection.points.size(); i++ )
	{
		detectionVector.block<2,1>(2*i,0) = Eigen::Vector2d( detection.points[i].x, detection.points[i].y );
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
		Point2D pt;
		pt.x = allPoints( 2*i );
		pt.y = allPoints( 2*i + 1 );
		out.points.push_back( pt );
	}
	return out;
}

isam::FiducialIntrinsics FiducialToIsam( const fiducials::Fiducial& fid )
{
	std::vector <isam::Point3d> pts;
	pts.reserve( fid.points.size() );
	for( unsigned int i = 0; i < fid.points.size(); i++ )
	{
		pts.push_back( MsgToIsam( fid.points[i] ) );
	}
	
	return isam::FiducialIntrinsics( pts );
}

fiducials::Fiducial IsamToFiducial( const isam::FiducialIntrinsics& fid )
{
	isam::FiducialIntrinsics::MatrixType fidMat;
	
	fiducials::Fiducial output;
	output.points.reserve( fidMat.cols() );
	for( unsigned int i = 0; i < fidMat.cols(); i++ )
	{
		geometry_msgs::Point pt;
		pt.x = fidMat(0,i);
		pt.y = fidMat(1,i);
		pt.z = fidMat(2,i);
		output.points.push_back( pt );
	}
	return output;
}

}
