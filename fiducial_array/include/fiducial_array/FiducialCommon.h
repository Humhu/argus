#pragma once

#include <opencv2/core/core.hpp>

#include <image_geometry/pinhole_camera_model.h>
#include "argus_msgs/FiducialDetection.h"
#include "argus_msgs/ImageFiducialDetections.h"

#include "fiducial_array/FiducialInfo.h"

namespace fiducial_array
{

/*! \brief Convert fiducial detections to OpenCV format. */
std::vector< cv::Point2f > MsgToPoints( const std::vector< argus_msgs::Point2D >& msg );
	
/*! \brief Convert fiducial intrinsics to OpenCV format. */
std::vector< cv::Point3f > MsgToPoints( const std::vector< geometry_msgs::Point >& msg );
	
/*! \brief Undistort and normalize fiducial detections in-place. Assumes all detections
 * have the same undistortion/normalization status. */
void UndistortDetections( const std::vector< argus_msgs::FiducialDetection >& detections,
                          const image_geometry::PinholeCameraModel& cameraModel, 
                          bool undistort, bool normalize,
                          std::vector< argus_msgs::FiducialDetection >& undistorted );
	
}
