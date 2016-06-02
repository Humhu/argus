#pragma once

#include "argus_utils/geometry/PoseSE3.h"

#include "fiducials/FiducialCommon.h"
#include "fiducials/FiducialArray.h"
#include "argus_msgs/FiducialDetection.h"
#include "sensor_msgs/CameraInfo.h"

namespace argus
{

/*! \brief Estimates the array pose using OpenCV's solvePnP. If camera info is null,
 * requires normalized and undistorted detections. Assumes standard camera convention
 * (z-forward) for imagePoints and object convention (x-forward) for input and returned
 * poses. */
PoseSE3 
EstimateArrayPose( const std::vector< cv::Point2f >& imagePoints,
                   const camplex::CameraCalibration* cameraModel,
                   const std::vector< cv::Point3f >& fiducialPoints,
                   const PoseSE3& guess = PoseSE3() );

}
