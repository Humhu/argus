#pragma once

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>

#include "argus_msgs/RelativePoseWithCovariance.h"
#include "fieldtrack/TargetInfoManager.h"

#include <unordered_map>

namespace argus
{

// TODO Make synchronized to allow threaded processing
// TODO Remove odom subs to make fixed frame
// TODO Don't track the reference frame
/*! \brief Tracks a set of targets relative to the reference frame. Subscribes
 * to relative poses, target body frame velocities, and odometry messages for the
 * observer. */
class FixedFrameTracker
{
public:

	FixedFrameTracker( ros::NodeHandle& nh, ros::NodeHandle& ph );
	
private:
	
	ros::NodeHandle nodeHandle;
	ros::NodeHandle privHandle;
	
	ros::Publisher targetPub;
	ros::Subscriber targetPoseSub;
	ros::Subscriber targetVelSub;
	std::shared_ptr<ros::Timer> timer;
	
	LookupInterface lookupInterface;
	TargetInfoManager targetManager;
	
	std::string referenceFrame;

	struct TargetRegistration
	{
		bool poseInitialized;
		ros::Time filterTime;
		ConstantVelocityFilterSE3 filter;
		ConstantVelocityFilterSE3::FullCovType Qrate;

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		TargetRegistration();
	};
	
	typedef std::unordered_map <std::string, TargetRegistration> TargetRegistry;
	TargetRegistry targetRegistry;
	
	void TimerCallback( const ros::TimerEvent& event );
	void TargetPoseCallback( const argus_msgs::RelativePoseWithCovariance::ConstPtr& msg );
	void TargetVelocityCallback( const geometry_msgs::TwistWithCovarianceStamped::ConstPtr& msg );
	
	void RegisterTarget( const std::string& name );
};
	
} // end namespace fieldtrack
