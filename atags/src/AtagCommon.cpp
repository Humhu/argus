#include "atags/AtagCommon.h"
#include "argus_utils/geometry/GeometryUtils.h"
#include <boost/foreach.hpp>

namespace argus 
{

FiducialDetection TagToFiducial( const AprilTags::TagDetection& tag,
                                 const std::string& family )
{
	FiducialDetection det;
	det.name = "apriltag_" + family + "_id" + std::to_string( tag.id );
	det.undistorted = false;
	det.normalized = false;
	det.points.reserve( 4 );
	for( unsigned int i = 0; i < 4; i++ )
	{
		det.points.emplace_back( tag.p[i].first, tag.p[i].second );
	}
	return det;
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
