#pragma once

#include <image_geometry/pinhole_camera_model.h>

#include "apriltags/TagDetection.h"
#include "argus_utils/geometry/PoseSE3.h"

#include "argus_msgs/FiducialDetection.h"

namespace argus 
{

/*! \brief Convert to FiducialDetection type. */
argus_msgs::FiducialDetection TagToFiducial( const AprilTags::TagDetection& tag,
                                             const std::string& family );

/*! \brief Returns transform from tag to camera assuming x-forward for both frames. */
// TODO Clean up this interface to use the msg.normalized member somehow
argus::PoseSE3 ComputeTagPose( const AprilTags::TagDetection& det, double tagSize,
                                     double fx, double fy, double px, double py );

/*! \brief Calculates the covariance of the corners to estimate skew. */
Eigen::Matrix2d ComputeCovariance( const AprilTags::TagDetection& det );

/*! \brief Returns the diagonal distances. */
std::pair<double, double> ComputeDiagonals( const AprilTags::TagDetection& det );

}
