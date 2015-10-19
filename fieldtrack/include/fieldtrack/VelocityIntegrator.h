#pragma once

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>

#include "argus_utils/PoseSE3.h"
#include "argus_msgs/RelativePose.h"

namespace fieldtrack
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
	ros::Publisher dispPub;
	std::shared_ptr<ros::Timer> timer;
	
	std::string referenceName;
	argus_utils::PoseSE3 integratedPose;
	bool initialized;
	
	void TimerCallback( const ros::TimerEvent& event );
	void TwistCallback( const geometry_msgs::TwistStamped::ConstPtr& msg );
	
};
	
} // end namespace fieldtrack
