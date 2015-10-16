#include "atags/AtagCommon.h"
#include "argus_utils/GeometryUtils.h"
#include <boost/foreach.hpp>

using namespace argus_utils;

namespace atags 
{
	
AprilTags::TagDetection MessageToDetection( const argus_msgs::TagDetection& msg )
{
	AprilTags::TagDetection det;
	det.good = true;
	det.id = msg.id;
	det.hammingDistance = msg.hammingDistance;
	double xacc = 0, yacc = 0;
	for( unsigned int i = 0; i < 4; i++ )
	{
		det.p[i].first = msg.corners[i].x;
		det.p[i].second = msg.corners[i].y;
		xacc += msg.corners[i].x;
		yacc += msg.corners[i].y;
	}
	det.cxy.first = xacc/4.0;
	det.cxy.second = yacc/4.0;
	
	// TODO Parameters we can't populate correctly?
	det.obsCode = 0;
	det.code = 0;
	det.observedPerimeter = 0.0;
	det.homography = Eigen::Matrix3d::Identity();
	det.hxy = std::pair<float,float>( 0.0, 0.0 );
	return det;
}

argus_msgs::TagDetection DetectionToMessage( const AprilTags::TagDetection& det,
                                             const std::string& family,
                                             bool undistorted, bool normalized )
{
	argus_msgs::TagDetection msg;
	msg.family = family;
	msg.id = det.id;
	msg.hammingDistance = det.hammingDistance;
	msg.undistorted = undistorted;
	msg.normalized = normalized;
	for( unsigned int i = 0; i < 4; i++ )
	{
		msg.corners[i].x = det.p[i].first;
		msg.corners[i].y = det.p[i].second;
	}
	return msg;
}

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

std::vector<AprilTags::TagDetection> 
MessageToDetections( const argus_msgs::TagDetectionsStamped& msg )
{
	std::vector<AprilTags::TagDetection> detections;
	detections.reserve( msg.detections.size() );
	for( unsigned int i = 0; i < msg.detections.size(); i++ )
	{
		detections.push_back( MessageToDetection( msg.detections[i] ) );
	}
	return detections;
}

argus_msgs::TagDetectionsStamped 
DetectionsToMessage( const std::vector<AprilTags::TagDetection>& detections, 
                     const std::string& family, const sensor_msgs::CameraInfo& info, 
                     bool undistorted, bool normalized)
{
	argus_msgs::TagDetectionsStamped msg;
	msg.detections.reserve( detections.size() );
	for( unsigned int i = 0; i < detections.size(); i++ )
	{
		msg.detections.push_back( DetectionToMessage( detections[i], family,
		                                              undistorted, normalized ) );
	}
	msg.info = info;
	return msg;
}

argus_utils::PoseSE3 ComputeTagPose( const AprilTags::TagDetection& det, double tagSize,
                                   double fx, double fy, double px, double py )
{
	PoseSE3 pose;
	Eigen::Vector3d translation;
	Eigen::Matrix3d rotation;
	
	det.getRelativeTranslationRotation( tagSize, fx, fy, px, py,
                                              translation, rotation );
	static PoseSE3 postrotation( PoseSE3::Translation( 0, 0, 0 ), 
                                 EulerToQuaternion( EulerAngles( -M_PI/2, -M_PI/2, 0 ) ) );
	
	PoseSE3::Translation t( translation );
	PoseSE3::Quaternion q( rotation );
	PoseSE3 H_tag_cam( t, q );
	return H_tag_cam * postrotation;
}

void UndistortDetections( const std::vector<AprilTags::TagDetection>& detections,
                          const image_geometry::PinholeCameraModel& cameraModel,
                          bool undistort, bool normalize, 
                          std::vector<AprilTags::TagDetection>& undistorted )
{
	cv::Mat detectedPoints( 4*detections.size(), 1, CV_64FC2 );
	
	unsigned int ind = 0;
	cv::Vec2d p;
	BOOST_FOREACH( const AprilTags::TagDetection& det, detections )
	{
		for( unsigned int i = 0; i < 4; i++ )
		{
			p[0] = det.p[i].first;
			p[1] = det.p[i].second;
			detectedPoints.at<cv::Vec2d>(ind,0) = p;
			ind++;
		}
	}
	
	cv::Mat undistortedMat, distortionCoeffs;
	cv::Matx33d outputCameraMatrix = cv::Matx33d::eye();
	
	if( undistort ) { distortionCoeffs = cameraModel.distortionCoeffs(); }
	if( normalize ) { outputCameraMatrix = cameraModel.intrinsicMatrix(); }
	
	cv::undistortPoints( detectedPoints, undistortedMat, cameraModel.intrinsicMatrix(),
	                     distortionCoeffs, cv::noArray(), outputCameraMatrix );
	
	ind = 0;
	undistorted.reserve( detections.size() );
	for( unsigned int j = 0; j < detections.size(); j++ )
	{
		AprilTags::TagDetection undistortedDetection;
		for( unsigned int i = 0; i < 4; i++ )
		{
			p = undistortedMat.at<cv::Vec2d>(ind,0);
			ind++;
			
			undistortedDetection.p[i].first = p[0];
			undistortedDetection.p[i].second = p[1];
		}
		undistorted.push_back( undistortedDetection );
	}
}
	
} // end namespace atags
