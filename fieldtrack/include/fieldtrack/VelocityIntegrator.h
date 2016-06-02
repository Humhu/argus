#pragma once

#include <ros/ros.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "argus_utils/geometry/PoseSE3.h"
#include "argus_msgs/RelativePose.h"

namespace argus
{

/*! \brief Integrates twist velocities to pose displacements at a fixed
 * rate. Single-threaded. */
class VelocityIntegrator
{
public:
	
	VelocityIntegrator( ros::NodeHandle& nh, ros::NodeHandle& ph );
	
private:
	
	ros::NodeHandle nodeHandle;
	ros::NodeHandle privHandle;
	
	ros::Subscriber twistSub;
	ros::Publisher dispPub; // Publishes geometry_msgs::PoseWithCovarianceStamped
	std::shared_ptr<ros::Timer> timer;
	
	std::string referenceName;
	PoseSE3 integratedPose;
	PoseSE3::CovarianceMatrix integratedCovariance;
	PoseSE3 offset;
	bool initialized;
	
	// Trapezoid integration scheme
	bool twistInitialized;
	geometry_msgs::TwistWithCovarianceStamped lastTwist;
	double scale;
	
	void TimerCallback( const ros::TimerEvent& event );
	void TwistCallback( const geometry_msgs::TwistWithCovarianceStamped::ConstPtr& msg );
	
};
	
} // end namespace fieldtrack
