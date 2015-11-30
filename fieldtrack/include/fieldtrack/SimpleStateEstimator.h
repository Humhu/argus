#pragma once

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>

#include "argus_utils/KalmanFilter.hpp"
#include "argus_utils/ManifoldKalmanFilter.hpp"
#include "argus_utils/PoseSE3.h"

namespace fieldtrack
{

/*! \brief Subscribes to displacement from odometers and absolute pose estimates
 * from localization sensors. Outputs nav_msgs::Odometry */
class SimpleStateEstimator
{
public:
	
	SimpleStateEstimator( const ros::NodeHandle& nh, const ros::NodeHandle& ph );
	
private:
	
	// 6D state, no controls, 6D observations
	typedef argus_utils::KalmanFilter<double, 6, Eigen::Dynamic, 6> VelocityFilter;
	VelocityFilter velocityFilter;
	VelocityFilter::StateCovariance velocityCovarianceRate;
	ros::Time lastVelocityUpdate;
	
	typedef argus_utils::ManifoldKalmanFilter<argus_utils::PoseSE3, argus_utils::BodyFrame> PoseFilter;
	PoseFilter poseFilter;
	PoseFilter::StateCovariance poseCovarianceRate;
	ros::Time lastPoseUpdate;
	
	ros::NodeHandle nodeHandle;
	ros::NodeHandle privHandle;
	
	std::string referenceFrame;
	std::string bodyFrame;
	std::shared_ptr<ros::Timer> updateTimer;
	
	ros::Subscriber dispSub; // Subscribes to geometry_msgs::PoseWithCovarianceStamped
	ros::Subscriber poseSub; // Subscribes to geometry_msgs::PoseWithCovarianceStamped
	ros::Publisher odomPub; // Publishes nav_msgs::Odometry
	
	void DisplacementCallback( const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg );
	void PoseCallback( const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg );
	void TimerCallback( const ros::TimerEvent& event );
	
};
	
}
