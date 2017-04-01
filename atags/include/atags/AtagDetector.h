#pragma once

#include <ros/ros.h>
#include <apriltags/TagDetector.h>

#include "camplex/FiducialCommon.h"
#include "camplex/CameraCalibration.h"

namespace argus
{

class AtagDetector
{
public:

	AtagDetector();

	void ReadParams( const ros::NodeHandle& ph );

	void SetUndistortion( bool enable );
	void SetNormalization( bool enable );
	void SetFamily( const std::string& family );
	void SetMaxSkewness( double s );
	void SetMinArea( double a );

	std::vector<FiducialDetection> ProcessImage( const cv::Mat& image,
	                                             const CameraCalibration& cal ) const;

private:

	AprilTags::TagDetector::Ptr _detector;
	std::string _family;
	bool _undistort;
	bool _normalize;
	double _maxSkewnessRatio;
	double _minAreaProduct;

	bool CheckDetection( const AprilTags::TagDetection& det ) const;
};

}