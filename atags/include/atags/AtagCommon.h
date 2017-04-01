#pragma once

#include <image_geometry/pinhole_camera_model.h>

#include "camplex/FiducialCommon.h"
#include "apriltags/TagDetection.h"
#include "argus_utils/geometry/PoseSE3.h"

#include "argus_msgs/FiducialDetection.h"

namespace argus 
{

/*! \brief Convert to FiducialDetection type. */
FiducialDetection TagToFiducial( const AprilTags::TagDetection& tag,
                                 const std::string& family );

/*! \brief Calculates the covariance of the corners to estimate skew. */
Eigen::Matrix2d ComputeCovariance( const AprilTags::TagDetection& det );

/*! \brief Returns the diagonal distances. */
std::pair<double, double> ComputeDiagonals( const AprilTags::TagDetection& det );

}
