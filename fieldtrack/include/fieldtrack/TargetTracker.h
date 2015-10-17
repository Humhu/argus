#pragma once

#include <ros/ros.h>

#include "argus_msgs/RelativePose.h"
#include "argus_utils/PoseSE3.h"
#include "argus_utils/ManifoldKalmanFilter.hpp"

#include <unordered_map>

namespace fieldtrack
{

// NOTE This class is not synchronized. Do not use with AsyncSpinner or threads.
/*! \brief Subscribes to relative pose information and fuses them to produce
 * pose estimates in the observer reference frame. */	
class TargetTracker
{
public:
	
	TargetTracker( ros::NodeHandle& nh, ros::NodeHandle& ph );
	
private:
	
	typedef argus_utils::ManifoldKalmanFilter< argus_utils::PoseSE3, argus_utils::WorldFrame > Filter;
	
	ros::NodeHandle nodeHandle;
	ros::NodeHandle privHandle;
	ros::Subscriber poseSub;
	
	bool firstIteration;
	
	std::shared_ptr<ros::Timer> timer;
	
	/*! \brief The observer frame name. */
	std::string referenceName;
	
	/*! \brief Map from target reference frame name to publisher. */
	std::unordered_map< std::string, ros::Publisher > publishers;
	
	/*! \brief Map from target reference frame name to corresponding filter. */
	std::unordered_map< std::string, Filter > filters;
	
	// TODO Load these in somehow
	/*! \brief Covariances for the filter. */
	Filter::StateCovariance initCovariance;
	Filter::StateCovariance transCovariance;
	Filter::ObservationCovariance obsCovariance;
	
	void TimerCallback( const ros::TimerEvent& event );
	void PoseCallback( const argus_msgs::RelativePose::ConstPtr& msg );
};
	
} // end namespace fieldtrack
