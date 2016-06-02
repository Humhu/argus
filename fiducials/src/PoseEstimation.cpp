#include "fiducials/PoseEstimation.h"
#include "argus_utils/geometry/GeometryUtils.h"

#include <opencv2/calib3d/calib3d.hpp>

using namespace argus_msgs;

namespace argus
{

// TODO if info is null, assume normalized and undistorted detections
PoseSE3 EstimateArrayPose( const std::vector< cv::Point2f >& imagePoints,
                           const camplex::CameraCalibration* cameraModel,
                           const std::vector< cv::Point3f >& fiducialPoints,
                           const PoseSE3& guess )
{
	cv::Matx33f cameraMat;
	cv::Mat distortionCoeffs;
	if( cameraModel != nullptr )
	{
		cameraMat = cameraModel->GetIntrinsicMatrix();
		distortionCoeffs = cameraModel->GetDistortionCoeffs();
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
	
	// TODO Figure out why guess causes it to fail
	cv::solvePnP( fiducialPoints, imagePoints, cameraMat, distortionCoeffs, rvec, tvec, false );
	
	cv::Rodrigues( rvec, R );
	H << R(0,0), R(0,1), R(0,2), tvec.at<double>(0),
	     R(1,0), R(1,1), R(1,2), tvec.at<double>(1),
	     R(2,0), R(2,1), R(2,2), tvec.at<double>(2),
	          0,      0,      0,      1;
	
	// Compensate for difference between x-forward and z-forward convention
	static PoseSE3 prerotation( 0, 0, 0, -0.5, 0.5, -0.5, 0.5 );
	
	return prerotation * PoseSE3( H );
}
	
}
