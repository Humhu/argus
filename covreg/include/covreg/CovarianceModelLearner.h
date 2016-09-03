#pragma once

#include "covreg/ClipOptimizer.h"

#include "percepto/optim/OptimizerTypes.h"
#include "argus_utils/synchronization/SynchronizationTypes.h"

#include <unordered_map>
#include <ros/ros.h>
#include <argus_msgs/FilterStepInfo.h>

namespace argus
{

class CovarianceModelLearner
{
public:

	CovarianceModelLearner();

	void Initialize( ros::NodeHandle& nh,
	                 ros::NodeHandle& ph );

private:
	struct EstimatorRegistration
	{
		ros::Publisher paramPublisher;
		CovarianceManager manager;
		std::string paramOutputPath;
		double weight;
	};
	typedef std::unordered_map<std::string, EstimatorRegistration> EstimatorRegistry;
	EstimatorRegistry _estRegistry;

	ros::Subscriber _infoSubscriber;
	ros::Timer _optimizerTimer;

	std::shared_ptr<ClipOptimizer> _optimizer;

	void InfoMsgCallback( const argus_msg::FilterStepInfo::ConstPtr& msg );

	void OptimizerTimerCallback( const ros::TimerEvent& event );
};

}