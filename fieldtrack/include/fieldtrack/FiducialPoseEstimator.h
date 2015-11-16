#pragma once

#include "argus_msgs/ImageFiducialDetections.h"

#include "fiducials/FiducialInfoManager.h"
#include "extrinsics_array/ExtrinsicsInfoManager.h"

namespace fieldtrack
{

/*! \brief Listens to fiducial detections and converts them to relative poses. */
class FiducialPoseEstimator
{
public:
	
	FiducialPoseEstimator( ros::NodeHandle& nh, ros::NodeHandle& ph );
	
private:
	
	/*! \brief Subscribes to /detections. Should be remapped. */
	ros::Subscriber detSub;
	
	ros::Publisher posePub;
	
	lookup::LookupInterface lookupInterface;
	fiducials::FiducialInfoManager fiducialManager;
	extrinsics_array::ExtrinsicsInfoManager extrinsicsManager;
	
	std::unordered_map<std::string, fiducials::Fiducial> transformedFiducials;
	
	void DetectionsCallback( const argus_msgs::ImageFiducialDetections::ConstPtr& msg ); 
	
	bool RetrieveCameraInfo( const std::string& cameraName );
	bool RetrieveFiducialInfo( const std::string& fidName );
	
};
	
} // end namespace fieldtrack
