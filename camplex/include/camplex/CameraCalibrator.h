#pragma once

#include "camplex/CameraCalibration.h"
#include "camplex/FiducialCommon.h"

namespace argus
{

/*! \brief Specifies the camera model complexity. */
struct CameraCalibrationParams
{
	bool optimizeAspectRatio;
	bool optimizePrincipalPoint;
	bool enableRadialDistortion[3];
	bool enableRationalDistortion[3];
	bool enableTangentialDistortion;
	bool enableThinPrism;

	CameraCalibrationParams();

	cv::Matx33d CreateIntrinsicsMatrix() const;
	cv::Mat CreateDistortionCoefficients() const;
	int CreateCalibrationFlags() const;
};
std::ostream& operator<<( std::ostream& os, const CameraCalibrationParams& params );

class CameraCalibrator
{
public:

	CameraCalibrator( const std::string& cameraName,
	                  const cv::Size& resolution,
					  const CameraCalibrationParams& params = CameraCalibrationParams() );

	void SetCalibrationParams( const CameraCalibrationParams& params );

	void AddDetection( const Fiducial& fid, const FiducialDetection& detection );

	CameraCalibration Calibrate() const;

private:

	std::string _cameraName;
	cv::Size _resolution;
	CameraCalibrationParams _params;
	
	typedef std::vector<cv::Point3f> CvPoints3f;
	typedef std::vector<cv::Point2f> CvPoints2f;

	std::vector<CvPoints3f> _fidPoints;
	std::vector<CvPoints2f> _imgPoints;
};

}