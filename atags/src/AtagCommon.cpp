#include "atags/AtagCommon.h"
#include "argus_utils/geometry/GeometryUtils.h"
#include <boost/foreach.hpp>

namespace argus 
{

argus_msgs::FiducialDetection TagToFiducial( const AprilTags::TagDetection& tag,
                                             const std::string& family )
{
	argus_msgs::FiducialDetection det;
	det.name = "apriltag_" + family + "_id" + std::to_string( tag.id );
	det.undistorted = false;
	det.normalized = false;
	det.points.reserve( 4 );
	for( unsigned int i = 0; i < 4; i++ )
	{
		argus_msgs::Point2D point;
		point.x = tag.p[i].first;
		point.y = tag.p[i].second;
		det.points.push_back( point );
	}
	return det;
}

PoseSE3 ComputeTagPose( const AprilTags::TagDetection& det, double tagSize,
                                   double fx, double fy, double px, double py )
{
	PoseSE3 pose;
	FixedVectorType<3> translation;
	FixedMatrixType<3,3> rotation;
	
	det.getRelativeTranslationRotation( tagSize, fx, fy, px, py,
	                                          translation, rotation );
	static PoseSE3 postrotation( Translation3Type( 0, 0, 0 ), 
	                             EulerToQuaternion( EulerAngles( -M_PI/2, -M_PI/2, 0 ) ) );
	
	Translation3Type t( translation );
	QuaternionType q( rotation );
	PoseSE3 H_tag_cam( t, q );
	return H_tag_cam * postrotation;
}

Eigen::Matrix2d ComputeCovariance( const AprilTags::TagDetection& det )
{
	Eigen::Vector2d points[4];
	for( unsigned int i = 0; i < 4; i++ )
	{
		points[i] << det.p[i].first, det.p[i].second;
	}
	
	Eigen::Vector2d mean = Eigen::Vector2d::Zero();
	for( unsigned int i = 0; i < 4; i++ )
	{
		mean += points[i];
	}
	mean *= 0.25;
	
	Eigen::Matrix2d cov = Eigen::Matrix2d::Zero();
	for( unsigned int i = 0; i < 4; i++ )
	{
		points[i] = points[i] - mean;
		cov += points[i] * points[i].transpose();
	}
	return cov * 0.25;
}

std::pair<double, double> ComputeDiagonals( const AprilTags::TagDetection& det )
{
	double dx1 = det.p[0].first - det.p[2].first;
	double dy1 = det.p[0].second - det.p[2].second;
	double dx2 = det.p[1].first - det.p[3].first;
	double dy2 = det.p[1].second - det.p[3].second;
	double dist1 = std::sqrt( dx1*dx1 + dy1*dy1 );
	double dist2 = std::sqrt( dx2*dx2 + dy2*dy2 );
	return std::pair<double,double>( dist1, dist2 );
}
	
} // end namespace atags
