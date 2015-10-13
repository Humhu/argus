#include "fiducial_array/PoseEstimation.h"

#include <opencv2/calib3d/calib3d.hpp>

namespace fiducial_array
{

// TODO if info is null, assume normalized and undistorted detections
argus_utils::PoseSE3 EstimateArrayPose( const std::vector< FiducialDetection >& detections,
                                        const image_geometry::PinholeCameraModel* cameraModel,
                                        const FiducialArray& array,
                                        const argus_utils::PoseSE3& guess )
{
	bool reqNormalized = ( cameraModel == nullptr );
	
	std::vector< cv::Point2f > points2d;
	std::vector< cv::Point3f > points3d;
	for( unsigned int i = 0; i < detections.size(); i++ )
	{
		if( reqNormalized && !detections[i].normalized )
		{
			throw std::runtime_error( "Required normalized detections but got unnormalized!" );
		}
		
		const std::vector< cv::Point3f >& ref = array.GetFiducialPoints( detections[i].name );
		points3d.insert( points3d.end(), ref.begin(), ref.end() );
		points2d.insert( points2d.end(), detections[i].points.begin(), detections[i].points.end() );
	}
	
	cv::Matx33f cameraMat;
	cv::Mat distortionCoeffs;
	if( cameraModel != nullptr )
	{
		cameraMat = cameraModel->intrinsicMatrix();
		distortionCoeffs = cameraModel->distortionCoeffs();
	}
	else
	{
		cameraMat = cv::Matx33f::eye();
	}
	
	// Initialize guess
	cv::Mat rvec;
	cv::Mat tvec( 3, 1, CV_64FC1 ); // Must allocate tvec
	cv::Matx33d R;
	Eigen::Matrix4d H = guess.ToTransform().matrix();
	for( unsigned int i = 0; i < 3; i++ )
	{
		for( unsigned int j = 0; j < 3; j++ )
		{
			R(i,j) = H(i,j);
		}
		tvec.at<double>(i) = H(i,3);
	}
	cv::Rodrigues( R, rvec );
	
	cv::solvePnP( points3d, points2d, cameraMat, distortionCoeffs, rvec, tvec, true );
	
	cv::Rodrigues( rvec, R );
	H << R(0,0), R(0,1), R(0,2), tvec.at<double>(0),
			R(1,0), R(1,1), R(1,2), tvec.at<double>(1),
			R(2,0), R(2,1), R(2,2), tvec.at<double>(2),
			0,      0,      0,      1;
	
	return argus_utils::PoseSE3( H );
}
	
}
