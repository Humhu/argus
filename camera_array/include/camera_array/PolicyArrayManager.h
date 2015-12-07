#pragma once

#include "camera_array/CameraArrayManager.h"
#include "nav_msgs/Odometry.h"

namespace camera_array
{

class PolicyArrayManager
: public CameraArrayManager
{
public:

	PolicyArrayManager( const ros::NodeHandle& nh, const ros::NodeHandle& ph );
	
protected:
	
	std::shared_ptr<ros::Timer> updateTimer;
	
		
	ros::Subscriber odomSub;
	nav_msgs::Odometry::ConstPtr lastOdometry;
	
	void OdometryCallback( const nav_msgs::Odometry::ConstPtr& msg );
	void TimerCallback( const ros::TimerEvent& event ) = 0;

};

}

