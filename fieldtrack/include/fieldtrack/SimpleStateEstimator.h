#pragma once

#include <ros/ros.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include "argus_msgs/RelativePoseWithCovariance.h"
#include <nav_msgs/Odometry.h>

#include "argus_utils/filters/FilterTypes.h"
#include "argus_utils/geometry/PoseSE3.h"

namespace argus
{

/*! \brief Subscribes to velocities from odometers and relative pose estimates
 * from localization sensors. Outputs nav_msgs::Odometry */
class SimpleStateEstimator
{
public:
	
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	SimpleStateEstimator( const ros::NodeHandle& nh, const ros::NodeHandle& ph );
	
private:
	
	ros::NodeHandle nodeHandle;
	ros::NodeHandle privHandle;
	
	typedef ConstantAccelFilterSE3 FilterType;
	FilterType filter;
	FilterType::FullCovType Qrate;
	ros::Time filterTime;

	bool twoDimensional;
	
	std::string referenceFrame;
	std::string bodyFrame;
	std::shared_ptr<ros::Timer> updateTimer;
	
	ros::Subscriber velSub; // Subscribes to geometry_msgs::TwistWithCovarianceStamped
	ros::Subscriber poseSub; // Subscribes to argus_msgs::RelativePose
	ros::Publisher odomPub; // Publishes nav_msgs::Odometry
	
	void VelocityCallback( const geometry_msgs::TwistWithCovarianceStamped::ConstPtr& msg );
	void PoseCallback( const argus_msgs::RelativePoseWithCovariance::ConstPtr& msg );
	void TimerCallback( const ros::TimerEvent& event );
	void EnforceTwoDimensionality();
};
	
}
