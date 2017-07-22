#pragma once

#include <opencv2/core.hpp>

#include "argus_utils/geometry/PoseSE2.h"
#include "camplex/CameraCalibration.h"

namespace argus
{

typedef cv::Point2d InterestPoint;
typedef std::vector<InterestPoint> InterestPoints;

typedef cv::Point2f InterestPointf;
typedef std::vector<InterestPointf> InterestPointsf;

std::ostream& operator<<( std::ostream& os, const InterestPoints& points );

InterestPointsf DowncastInterestPoints( const InterestPoints& in );
InterestPoints UpcastInterestPoints( const InterestPointsf& in );

// TODO Move into a utils package for open cv? Shared functionality with fiducials package
InterestPoints UndistortPoints( const InterestPoints& points, 
                                const CameraCalibration& model,
                                bool undistort, 
                                bool normalize );

InterestPoints TransformPoints( const InterestPoints& points,
                                const PoseSE2& trans );

struct FrameInterestPoints
{
	ros::Time time;
	cv::Mat frame;
	InterestPoints points;
	CameraCalibration cameraModel;

	FrameInterestPoints Undistort() const;
	FrameInterestPoints Normalize() const;
	FrameInterestPoints Unnormalize() const;
	FrameInterestPoints UndistortAndNormalize() const;
};

}
