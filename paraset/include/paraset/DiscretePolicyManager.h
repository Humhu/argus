#pragma once

#include "paraset/DiscretePolicy.h"
#include "paraset/PolicyModules.h"
#include "broadcast/BroadcastMultiReceiver.h"

#include <boost/random/mersenne_twister.hpp>

namespace argus
{

class DiscretePolicyManager
{
public:

	DiscretePolicyManager( ros::NodeHandle& ph );

private:

	DiscretePolicy _policyInterface;
	
	NormalizedPerceptron::Ptr _network;
	percepto::TerminalSource<VectorType> _networkInput;
	percepto::Parameters::Ptr _networkParameters;
	
	BroadcastMultiReceiver _inputStreams;
	
	boost::mt19937 _engine;
	ros::Timer _timer;

	void UpdateCallback( const ros::TimerEvent& event );
};

}