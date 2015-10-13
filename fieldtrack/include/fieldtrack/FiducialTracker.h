#pragma once

#include "argus_msgs/ImageFiducialDetections.h"

#include "fiducial_array/FiducialCommon.h"
#include "fiducial_array/GetFiducialArrayInfo.h"

#include "camera_array/CameraArray.h"
#include "camera_array/GetCameraArrayInfo.h"

namespace fieldtrack
{

/*! \brief Tracks an array of fiducials. */
class FiducialTracker
{
public:
	
	FiducialTracker( ros::NodeHandle& nh, ros::NodeHandle& ph );
	
private:
	
	ros::Subscriber detSub;
	
	void DetectionsCallback( const argus_msg::ImageFiducialDetections::ConstPtr& msg ); 
	
};
	
};
