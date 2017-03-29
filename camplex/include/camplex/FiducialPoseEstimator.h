#pragma once

#include "argus_utils/geometry/PoseSE3.h"
#include "argus_msgs/ImageFiducialDetections.h"

#include "camplex/FiducialCommon.h"
#include "camplex/FiducialInfoManager.h"
#include "extrinsics_array/ExtrinsicsInterface.h"

namespace argus
{

// TODO Be able to publish multiple ref frame observations
/*! \brief Listens to fiducial detections and converts them to relative poses. */
class FiducialPoseEstimator
{
public:
	
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	FiducialPoseEstimator( ros::NodeHandle& nh, ros::NodeHandle& ph );
	
private:
	
	std::string _refFrame;
	ros::Subscriber _detSub;
	ros::Publisher _posePub;
	
	LookupInterface _lookupInterface;
	FiducialInfoManager _fiducialManager;
	ExtrinsicsInterface _extrinsicsInterface;
	
	std::unordered_map<std::string, Fiducial> _transformedFiducials;
	
	// Gets the specified fiducial transformed into _refFrame at the specified time
	bool GetFiducial( const std::string& name, const ros::Time& time, Fiducial& fid );
	void DetectionsCallback( const argus_msgs::ImageFiducialDetections::ConstPtr& msg ); 
	
	bool RetrieveFiducialInfo( const std::string& fidName );
	
};
	
} // end namespace fieldtrack
