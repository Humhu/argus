#pragma once

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include "argus_msgs/CompactOdometry.h"
#include "argus_msgs/CompactOdometryArray.h"
#include "argus_msgs/RelativePoseWithCovariance.h"

#include "fieldtrack/TargetInfoManager.h"
#include "argus_utils/synchronization/SynchronizationTypes.h"

#include <unordered_map>

namespace argus
{

// TODO Subscribe to body-velocity estimates for targets?
/*! \brief Tracks independent targets relative to a moving reference frame. 
 * Publishes the state of the targets in compact odometry message arrays. 
 * Subscribes to relative pose observations. */
class BodyFrameTracker
{
public:

	BodyFrameTracker( const ros::NodeHandle& nh, const ros::NodeHandle& ph );
	
private:
	
	ros::NodeHandle nodeHandle;
	ros::NodeHandle privHandle;
	
	std::string referenceFrame;
	LookupInterface lookupInterface;
	TargetInfoManager targetManager;
	
	ros::Publisher targetPub;
	ros::Subscriber refOdomSub;
	ros::Subscriber targetPoseSub;
	
	std::shared_ptr<ros::Timer> updateTimer;
	
	struct TargetRegistration
	{
		bool poseInitialized;
		ros::Time filterTime;
		ConstantVelocityFilterSE3 filter;
		ConstantVelocityFilterSE3::FullCovType Qrate;

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		TargetRegistration();
	};
	typedef std::unordered_map<std::string, TargetRegistration> TargetRegistry;
	TargetRegistry targetRegistry;
	
	mutable argus::Mutex mutex;
	nav_msgs::Odometry::ConstPtr lastOdometry;
	
	void TimerCallback( const ros::TimerEvent& event );
	void OdometryCallback( const nav_msgs::Odometry::ConstPtr& msg );
	void TargetPoseCallback( const argus_msgs::RelativePoseWithCovariance::ConstPtr& msg );
	
	void RegisterTarget( const std::string& name );
};
	
}
