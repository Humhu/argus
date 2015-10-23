#pragma once

#include <image_geometry/pinhole_camera_model.h>

#include "apriltags/TagDetection.h"
#include "argus_utils/PoseSE3.h"

#include "argus_msgs/TagDetection.h"
#include "argus_msgs/TagDetectionsStamped.h"
#include "argus_msgs/FiducialDetection.h"

namespace atags 
{

/*! \brief Convert to and from a tag detection message. */
AprilTags::TagDetection MessageToDetection( const argus_msgs::TagDetection& msg );
argus_msgs::TagDetection DetectionToMessage( const AprilTags::TagDetection& det,
                                             const std::string& family,
                                             bool undistorted, bool normalized );

/*! \brief Convert to and from a tag-image detections message. */
std::vector<AprilTags::TagDetection> MessageToDetections( const argus_msgs::TagDetectionsStamped& msg );

argus_msgs::TagDetectionsStamped 
DetectionsToMessage( const std::vector<AprilTags::TagDetection>& detections, 
                     const std::string& family, const sensor_msgs::CameraInfo& info, 
                     bool undistorted, bool normalized);

/*! \brief Convert to FiducialDetection type. */
argus_msgs::FiducialDetection TagToFiducial( const AprilTags::TagDetection& tag,
                                             const std::string& family );

/*! \brief Returns transform from tag to camera assuming x-forward for both frames. */
// TODO Clean up this interface to use the msg.normalized member somehow
argus_utils::PoseSE3 ComputeTagPose( const AprilTags::TagDetection& det, double tagSize,
                                     double fx, double fy, double px, double py );

/*! \brief Calculates the covariance of the corners to estimate skew. */
Eigen::Matrix2d ComputeCovariance( const AprilTags::TagDetection& det );

/*! \brief Returns the diagonal distances. */
std::pair<double, double> ComputeDiagonals( const AprilTags::TagDetection& det );

}
