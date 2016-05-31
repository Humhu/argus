#pragma once

#include <opencv2/core.hpp>
#include <image_geometry/pinhole_camera_model.h>

namespace odoflow
{

typedef cv::Point2d InterestPoint;
typedef std::vector<InterestPoint> InterestPoints;

// TODO Move into a utils package for open cv? Shared functionality with fiducials package
bool UndistortPoints( const InterestPoints& points, 
                      const image_geometry::PinholeCameraModel& model,
                      bool undistort, bool normalize,
                      InterestPoints& undistorted );

}
