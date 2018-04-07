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
                                const CameraCalibration& model );
InterestPoints NormalizePoints( const InterestPoints& points,
                                const CameraCalibration& model );
InterestPoints UndistortAndNormalizePoints( const InterestPoints& points,
                                            const CameraCalibration& model );

/*! \brief Distort and unnormalize points that are normalized and undistorted */
InterestPoints DistortPoints( const InterestPoints& points,
                              const CameraCalibration& model );
InterestPoints UnnormalizePoints( const InterestPoints& points,
                                  const CameraCalibration& model );
InterestPoints DistortAndUnnormalizePoints( const InterestPoints& points,
                                            const CameraCalibration& model );

InterestPoints TransformPoints( const InterestPoints& points,
                                const PoseSE2& trans );
}
