#pragma once

#include "argus_utils/PoseSE3.h"

#include "fiducials/FiducialCommon.h"
#include "fiducials/FiducialArray.h"
#include "argus_msgs/FiducialDetection.h"
#include "sensor_msgs/CameraInfo.h"

namespace fiducials
{

// TODO Camera model or camera info?
/*! \brief Estimates the array pose using OpenCV's solvePnP. If camera info is null,
 * requires normalized and undistorted detections. */
argus_utils::PoseSE3 EstimateArrayPose( const std::vector< cv::Point2f >& imagePoints,
                                        const image_geometry::PinholeCameraModel* cameraModel,
                                        const std::vector< cv::Point3f >& fiducialPoints,
                                        const argus_utils::PoseSE3& guess = argus_utils::PoseSE3() );


	
}