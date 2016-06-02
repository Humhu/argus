#pragma once

#include <ros/ros.h>

#include "argus_msgs/RelativePose.h"

#include "argus_utils/geometry/PoseSE3.h"
#include "extrinsics_array/ExtrinsicsInfoManager.h"

#include <unordered_map>

namespace argus
{
	
/*! \brief Estimates the pose of a frame relative to arrays by fusing observations
 * of the relative pose to the array members. Uses the extrinsics lookup interface.
 * Subscribes to relative_poses and publishes on */
class ArrayPoseEstimator
{
public:
	
	ArrayPoseEstimator( ros::NodeHandle& nh, ros::NodeHandle& ph );
	
private:
	
	ros::NodeHandle nodeHandle;
	ros::NodeHandle privHandle;
	
	LookupInterface lookupInterface;
	ExtrinsicsInfoManager extrinsicsManager;
	
	/*! \brief The frame this estimator tracks relative to arrays. */
	std::string referenceName;
	
	ros::Subscriber relPoseSub;
	std::unordered_map< std::string, ros::Publisher > relPosePubs;
	
	void RelativePoseCallback( const argus_msgs::RelativePose::ConstPtr& msg );
};
	
} // end namespace fieldtrack
