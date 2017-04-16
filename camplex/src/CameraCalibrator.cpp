#include "camplex/CameraCalibrator.h"

#include <opencv2/calib3d/calib3d.hpp>

namespace argus
{

CameraCalibrationParams::CameraCalibrationParams()
	: optimizeAspectRatio( false ),
	optimizePrincipalPoint( false ),
	enableRadialDistortion { false, false, false },
    enableRationalDistortion { false, false, false },
    enableTangentialDistortion( false ),
    enableThinPrism( false ) {}

cv::Matx33d CameraCalibrationParams::CreateIntrinsicsMatrix() const
{
	return cv::Matx33d::eye();
}

// NOTE OpenCV's calibration routine expects a Mat of a certain size
// depending on the flags. This does not mean they will all be populated!
cv::Mat CameraCalibrationParams::CreateDistortionCoefficients() const
{
	unsigned int numDistortionParams = 4;

	if( enableRadialDistortion[2] ) { numDistortionParams = 5; }

	if( enableRationalDistortion[0] ||
	    enableRationalDistortion[1] ||
	    enableRationalDistortion[2] ) { numDistortionParams = 8; }

	if( enableThinPrism ) { numDistortionParams = 12; }

	return cv::Mat::zeros( numDistortionParams, 1, CV_64F );
}

int CameraCalibrationParams::CreateCalibrationFlags() const
{
	int flags = 0;

	if( !optimizeAspectRatio ) { flags |= cv::CALIB_FIX_ASPECT_RATIO; }
	if( !optimizePrincipalPoint ) { flags |= cv::CALIB_FIX_PRINCIPAL_POINT; }

	if( !enableRadialDistortion[0] ) { flags |= cv::CALIB_FIX_K1; }
	if( !enableRadialDistortion[1] ) { flags |= cv::CALIB_FIX_K2; }
	if( !enableRadialDistortion[2] ) { flags |= cv::CALIB_FIX_K3; }

	if( enableRationalDistortion[0] ||
	    enableRationalDistortion[1] ||
	    enableRationalDistortion[2] )
	{
		flags |= cv::CALIB_RATIONAL_MODEL;
		if( !enableRationalDistortion[0] ) { flags |= cv::CALIB_FIX_K4; }
		if( !enableRationalDistortion[1] ) { flags |= cv::CALIB_FIX_K5; }
		if( !enableRationalDistortion[2] ) { flags |= cv::CALIB_FIX_K6; }
	}

	if( !enableTangentialDistortion ) { flags |= cv::CALIB_ZERO_TANGENT_DIST; }
	if( enableThinPrism ) { flags |= cv::CALIB_THIN_PRISM_MODEL; }

	return flags;
}

CameraCalibrator::CameraCalibrator( const std::string& cameraName,
                                    const cv::Size& resolution,
                                    const CameraCalibrationParams& params )
	: _cameraName( cameraName ), _resolution( resolution ), _params( params ) {
}

void CameraCalibrator::SetCalibrationParams( const CameraCalibrationParams& params )
{
	_params = params;
}

void CameraCalibrator::AddDetection( const Fiducial& fid,
                                     const FiducialDetection& detection )
{
	_fidPoints.push_back( PointsToCv( fid.points ) );
	_imgPoints.push_back( PointsToCv( detection.points ) );
}

CameraCalibration
CameraCalibrator::Calibrate() const
{
	cv::Matx33d intrinsics = _params.CreateIntrinsicsMatrix();
	cv::Mat distortion = _params.CreateDistortionCoefficients();
	int flags = _params.CreateCalibrationFlags();

	std::vector<cv::Mat> rvecs, tvecs;
	cv::calibrateCamera( _fidPoints, _imgPoints, _resolution, 
	                     intrinsics, distortion,
	                     rvecs, tvecs, flags );

	return CameraCalibration( _cameraName, _resolution, intrinsics, distortion );
}

}
