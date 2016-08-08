#pragma once

#include "paraset/DiscretePolicy.h"
#include "paraset/PolicyModules.h"
#include "argus_msgs/FloatVectorStamped.h"
#include "broadcast/BroadcastMultiReceiver.h"

#include <boost/random/mersenne_twister.hpp>

namespace argus
{

// TODO Subscribe to parameter updates
class DiscretePolicyManager
{
public:

	DiscretePolicyManager();

	void Initialize( ros::NodeHandle& nh, ros::NodeHandle& ph );

private:

	DiscretePolicy _policyInterface;
	ros::Publisher _actionPub;
	ros::Subscriber _paramSub;
	
	NormalizedPerceptron::Ptr _network;
	percepto::TerminalSource<VectorType> _networkInput;
	percepto::Parameters::Ptr _networkParameters;
	
	BroadcastMultiReceiver _inputStreams;
	
	boost::mt19937 _engine;
	ros::Timer _timer;

	void UpdateCallback( const ros::TimerEvent& event );
	void ParamCallback( const argus_msgs::FloatVectorStamped::ConstPtr& msg );
};

}