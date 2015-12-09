#pragma once

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include "argus_msgs/CompactOdometry.h"
#include "argus_msgs/CompactOdometryArray.h"
#include "argus_msgs/RelativePoseWithCovariance.h"

#include "fieldtrack/ConstantVelocityFilter.h"

#include "argus_utils/SynchronizationUtils.h"

#include <unordered_map>

namespace fieldtrack
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
	
	ros::Publisher targetPub;
	ros::Subscriber refOdomSub;
	ros::Subscriber targetPoseSub;
	
	std::shared_ptr<ros::Timer> updateTimer;
	
	struct TargetRegistration
	{
		// ros::Time lastUpdate; // TODO Use somehow
		bool poseInitialized;
		ConstantVelocityFilter::Ptr filter;
	};
	typedef std::unordered_map<std::string, TargetRegistration> TargetRegistry;
	TargetRegistry targetRegistry;
	
	mutable argus_utils::Mutex mutex;
	nav_msgs::Odometry::ConstPtr lastOdometry;
	
	void TimerCallback( const ros::TimerEvent& event );
	void OdometryCallback( const nav_msgs::Odometry::ConstPtr& msg );
	void TargetPoseCallback( const argus_msgs::RelativePoseWithCovariance::ConstPtr& msg );
	
	void RegisterTarget( const std::string& name );
};
	
}
