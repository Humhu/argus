#pragma once

#include <image_geometry/pinhole_camera_model.h>
#include "argus_msgs/FiducialDetection.h"
#include "argus_msgs/ImageFiducialDetections.h"

namespace fiducial_array
{
	
struct FiducialDetection
{
	std::string name;
	bool undistorted;
	bool normalized;
	std::vector< cv::Point2f > points;
};

/*! \brief Convert to and from a fiducial detection message. */
FiducialDetection MsgToDetection( const argus_msgs::FiducialDetection& msg );
argus_msgs::FiducialDetection DetectionToMsg( const FiducialDetection& det );

/*! \brief Convert to and from an image fiducial detection message. */
argus_msgs::ImageFiducialDetections 
DetectionsToMsg( const std::vector< FiducialDetection >& detections );

/*! \brief Undistort and normalize fiducial detections in-place. Assumes all detections
 * have the same undistortion/normalization status. */
void UndistortDetections( const std::vector< FiducialDetection >& detections,
                          const image_geometry::PinholeCameraModel& cameraModel, 
                          bool undistort, bool normalize,
                          std::vector< FiducialDetection >& undistorted );
	
}
