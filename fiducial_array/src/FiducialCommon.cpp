#include "fiducial_array/FiducialCommon.h"
#include <boost/foreach.hpp>

namespace fiducial_array
{

std::vector< cv::Point2f > MsgToPoints( const std::vector< argus_msgs::Point2D >& msg )
{
	std::vector< cv::Point2f > points;
	points.reserve( msg.size() );
	BOOST_FOREACH( const argus_msgs::Point2D& p, msg)
	{
		cv::Point2f pc( p.x, p.y );
		points.push_back( pc );
	}
	return points;
}
	
std::vector< cv::Point3f > MsgToPoints( const std::vector< geometry_msgs::Point >& msg )
{
	std::vector< cv::Point3f > points;
	points.reserve( msg.size() );
	BOOST_FOREACH( const geometry_msgs::Point& p, msg)
	{
		cv::Point3f pc( p.x, p.y, p.z );
		points.push_back( pc );
	}
	return points;
}
	
void UndistortDetections( const std::vector< argus_msgs::FiducialDetection >& detections,
                          const image_geometry::PinholeCameraModel& cameraModel, 
                          bool undistort, bool normalize,
                          std::vector< argus_msgs::FiducialDetection >& undistorted )
{
	// Shortcut work if possible
	if( detections[0].undistorted == undistort 
	    && detections[0].normalized == normalize )
	{ return; }
	
	// Quick catch of uncalibrated camera model
	if( std::isnan( cameraModel.fx() ) ) 
	{
		throw std::runtime_error( "Attempted to undistort with uncalibrated camera model." );
	}
	
	// TODO Preallocate size?
	std::vector< cv::Point2f > points2d, undistortedPoints;
	for( unsigned int i = 0; i < detections.size(); i++ )
	{
		for( unsigned int j = 0; j < detections[i].points.size(); j++ )
		{
			cv::Point2f point( detections[i].points[j].x, detections[i].points[j].y );
			points2d.push_back( point );
		}
	}
	
	// Perform undistortion
	cv::Mat distortionCoeffs;
	cv::Matx33d outputCameraMatrix;
	
	if( undistort ) { distortionCoeffs = cameraModel.distortionCoeffs(); }
	outputCameraMatrix = normalize ? cv::Matx33d::eye() : cameraModel.intrinsicMatrix();
	
	cv::undistortPoints( points2d, undistortedPoints, cameraModel.intrinsicMatrix(),
	                     distortionCoeffs, cv::noArray(), outputCameraMatrix );
	
	// Assign to undistorted output 
	undistorted.clear();
	undistorted.reserve( detections.size() );
	unsigned int ind = 0;
	argus_msgs::FiducialDetection det;
	for( unsigned int i = 0; i < detections.size(); i++ )
	{
		det.name = detections[i].name;
		det.undistorted = undistort;
		det.normalized = normalize;
		det.points.reserve( detections[i].points.size() );
		for( unsigned int j = 0; j < detections[i].points.size(); j++ )
		{
			argus_msgs::Point2D point;
			point.x = undistortedPoints[ind].x;
			point.y = undistortedPoints[ind].y;
			det.points.push_back( point );
			ind++;
		}
		undistorted.push_back( det );
	}
}

} // end namespace fiducial_array
