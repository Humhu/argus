#pragma once

#include "argus_msgs/ImageFiducialDetections.h"

#include "fiducial_array/FiducialInfoManager.h"
#include "extrinsics_array/ExtrinsicsInfoManager.h"

namespace fieldtrack
{

/*! \brief Listens to fiducial detections and converts them to array relative poses. */
class ArrayPoseEstimator
{
public:
	
	ArrayPoseEstimator( ros::NodeHandle& nh, ros::NodeHandle& ph );
	
private:
	
	/*! \brief Subscribes to /detections. Should be remapped. */
	ros::Subscriber detSub;
	
	ros::Publisher posePub;
	
	fiducial_array::FiducialInfoManager fidManager;
	extrinsics_array::ExtrinsicsInfoManager camManager;
	
	void DetectionsCallback( const argus_msgs::ImageFiducialDetections::ConstPtr& msg ); 
	
};
	
} // end namespace fieldtrack
