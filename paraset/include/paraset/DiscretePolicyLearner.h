#pragma once

#include "paraset/DiscretePolicyManager.h"
#include "paraset/RewardStamped.h"
#include "paraset/DiscreteParamAction.h"
#include "paraset/PolicyLogGradientModules.h"

namespace argus
{

class DiscretePolicyLearner
{
public:

	typedef DiscretePolicyManager::NetworkType NetworkType;

	DiscretePolicyLearner( ros::NodeHandle& nh, ros::NodeHandle& ph );

private:

	percepto::Parameters::Ptr _networkParams;

	ros::Subscriber _actionSub;
	ros::Subscriber _rewardSub;
	ros::Publisher _paramPub;

	ros::Timer _updateTimer;

	void ActionCallback( const paraset::DiscreteParamAction::ConstPtr& msg );
	void RewardCallback( const paraset::RewardStamped::ConstPtr& msg );
	void TimerCallback( const ros::TimerEvent& event );
};

}