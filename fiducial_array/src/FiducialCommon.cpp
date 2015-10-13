#include "fiducial_array/FiducialCommon.h"

namespace fiducial_array
{

FiducialDetection MsgToDetection( const argus_msgs::FiducialDetection& msg )
{
	FiducialDetection det;
	det.name = msg.name;
	det.undistorted = msg.undistorted;
	det.normalized = msg.normalized;
	det.points.reserve( msg.points.size() );
	for( unsigned int i = 0; i < msg.points.size(); i++ )
	{
		cv::Point2f p( msg.points[i].x, msg.points[i].y );
		det.points.push_back( p );
	}
	return det;
}

argus_msgs::FiducialDetection DetectionToMsg( const FiducialDetection& det )
{
	argus_msgs::FiducialDetection msg;
	msg.name = det.name;
	msg.undistorted = det.undistorted;
	msg.normalized = det.normalized;
	msg.points.reserve( det.points.size() );
	for( unsigned int i = 0; i < det.points.size(); i++ )
	{
		argus_msgs::Point2D p;
		p.x = det.points[i].x;
		p.y = det.points[i].y;
		msg.points.push_back( p );
	}
	return msg;
}

argus_msgs::ImageFiducialDetections
DetectionsToMsg( const std::vector< FiducialDetection >& detections )
{
	argus_msgs::ImageFiducialDetections msg;
	msg.detections.reserve( detections.size() );
	for( unsigned int i = 0; i < detections.size(); i++ )
	{
		msg.detections.push_back( DetectionToMsg( detections[i] ) );
	}
	return msg;
}

void UndistortDetections( const std::vector< FiducialDetection >& detections,
                          const image_geometry::PinholeCameraModel& cameraModel, 
                          bool undistort, bool normalize,
                          std::vector< FiducialDetection >& undistorted )
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
		points2d.insert( points2d.end(), detections[i].points.begin(), detections[i].points.end() );
	}
	
	// Perform undistortion
	cv::Mat distortionCoeffs;
	cv::Matx33d outputCameraMatrix;
	
	if( undistort ) { distortionCoeffs = cameraModel.distortionCoeffs(); }
	outputCameraMatrix = normalize ? cameraModel.intrinsicMatrix() : cv::Matx33d::eye();
	
	cv::undistortPoints( points2d, undistortedPoints, cameraModel.intrinsicMatrix(),
	                     distortionCoeffs, cv::noArray(), outputCameraMatrix );
	
	// Assign to undistorted output 
	undistorted.reserve( detections.size() );
	unsigned int ind = 0;
	FiducialDetection det;
	for( unsigned int i = 0; i < detections.size(); i++ )
	{
		unsigned int numDetections = detections[i].points.size();
		det.name = detections[i].name;
		det.undistorted = undistort;
		det.normalized = normalize;
		det.points = std::vector< cv::Point2f >( undistortedPoints.begin() + ind,
		                                         undistortedPoints.begin() + ind + numDetections );
		ind += numDetections;
		undistorted.push_back( det );
	}
}

} // end namespace fiducial_array
