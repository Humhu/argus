#include "paraset/DiscretePolicyLearner.h"
#include "argus_msgs/FloatVectorStamped.h"
#include "argus_utils/utils/ParamUtils.h"

using namespace argus_msgs;
using namespace paraset;

namespace argus
{

DiscretePolicyLearner::DiscretePolicyLearner( ros::NodeHandle& nh, 
                                              ros::NodeHandle& ph ) 
{
	_paramPub = ph.advertise<FloatVectorStamped>( "param_updates", 1 );
	_actionSub = nh.subscribe( "actions", 
	                           0, 
	                           &DiscretePolicyLearner::ActionCallback, 
	                           this );
	_rewardSub = nh.subscribe( "rewards",
	                           0, 
	                           &DiscretePolicyLearner::RewardCallback,
	                           this );

	double updateRate;
	GetParamRequired( ph, "update_rate", updateRate );
	_updateTimer = nh.createTimer( ros::Duration( 1.0/updateRate ),
	                               &DiscretePolicyLearner::TimerCallback,
	                               this );
}

void DiscretePolicyLearner::ActionCallback( const DiscreteParamAction::ConstPtr& msg )
{
	// Record the action
}

void DiscretePolicyLearner::RewardCallback( const RewardStamped::ConstPtr& msg )
{

}

void DiscretePolicyLearner::TimerCallback( const ros::TimerEvent& event )
{

}

}