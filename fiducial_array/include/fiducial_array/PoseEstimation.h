#pragma once

#include "argus_utils/PoseSE3.h"

#include "fiducial_array/FiducialCommon.h"
#include "fiducial_array/FiducialArray.h"

namespace fiducial_array
{

/*! \brief Estimates the array pose using OpenCV's solvePnP. If model is null,
 * assumes normalized and undistorted detections. */
argus_utils::PoseSE3 EstimateArrayPose( const std::vector< FiducialDetection >& detections,
                                        const image_geometry::PinholeCameraModel* cameraModel,
                                        const FiducialArray& array,
                                        const argus_utils::PoseSE3& guess = argus_utils::PoseSE3() );


	
}
