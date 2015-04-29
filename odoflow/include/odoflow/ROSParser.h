#ifndef _OFLOW_ROS_PARSER_H_
#define _OFLOW_ROS_PARSER_H_

#include "odoflow/InterestPointDetector.h"
#include "odoflow/InterestPointTracker.h"
#include "odoflow/MotionEstimator.h"

#include <ros/ros.h>

namespace odoflow
{

	InterestPointDetector::Ptr ParseDetector( ros::NodeHandle& h );
	
	InterestPointTracker::Ptr ParseTracker( ros::NodeHandle& h );
	
	MotionEstimator::Ptr ParseEstimator( ros::NodeHandle& h );
	
}

#endif
