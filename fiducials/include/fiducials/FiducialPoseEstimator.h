#pragma once

#include "argus_utils/geometry/PoseSE3.h"
#include "argus_msgs/ImageFiducialDetections.h"

#include "fiducials/FiducialInfoManager.h"
#include "extrinsics_array/ExtrinsicsInfoManager.h"

namespace argus
{

/*! \brief Listens to fiducial detections and converts them to relative poses. */
class FiducialPoseEstimator
{
public:
	
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	FiducialPoseEstimator( ros::NodeHandle& nh, ros::NodeHandle& ph );
	
private:
	
	/*! \brief Subscribes to /detections. Should be remapped. */
	ros::Subscriber detSub;
	
	ros::Publisher posePub;
	
	LookupInterface lookupInterface;
	FiducialInfoManager fiducialManager;
	ExtrinsicsInfoManager extrinsicsManager;
	
	// TODO
	argus::PoseSE3::CovarianceMatrix covariance;
	
	std::unordered_map<std::string, Fiducial> transformedFiducials;
	
	void DetectionsCallback( const argus_msgs::ImageFiducialDetections::ConstPtr& msg ); 
	
	bool RetrieveCameraInfo( const std::string& cameraName );
	bool RetrieveFiducialInfo( const std::string& fidName );
	
};
	
} // end namespace fieldtrack
