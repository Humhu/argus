#include "paraset/DiscretePolicyManager.h"
#include "paraset/DiscreteParamAction.h"

#include "argus_utils/random/RandomUtils.hpp"
#include "argus_utils/utils/MatrixUtils.h"
#include "argus_utils/utils/MathUtils.h"

#include "percepto/utils/Randomization.hpp"

#include <boost/random/random_device.hpp>

using namespace argus_msgs;

namespace argus
{

DiscretePolicyManager::DiscretePolicyManager() {}

void DiscretePolicyManager::Initialize( ros::NodeHandle& nh,
                                        ros::NodeHandle& ph )
{
	_policyInterface.Initialize( ph );

	_actionPub = ph.advertise<DiscreteParamAction::MsgType>( "actions", 0 );
	_paramSub = nh.subscribe( "param_updates", 
	                          1,
	                          &DiscretePolicyManager::ParamCallback, 
	                          this );

	ros::NodeHandle subh( ph.resolveName("input_streams") );
	_inputStreams.Initialize( subh );

	unsigned int inputDim = _inputStreams.GetDim();
	unsigned int outputDim = _policyInterface.GetNumCombinations();
	unsigned int numHiddenLayers, layerWidth;
	GetParamRequired( ph, "network/num_hidden_layers", numHiddenLayers );
	GetParamRequired( ph, "network/layer_width", layerWidth );
	_network = std::make_shared<NetworkType>( inputDim, 
	                                          outputDim, 
	                                          numHiddenLayers, 
	                                          layerWidth );
	_network->SetSource( &_networkInput );
	_networkParameters = _network->CreateParameters();
	VectorType w = _networkParameters->GetParamsVec();
	percepto::randomize_vector( w, -0.1, 0.1 );
	_networkParameters->SetParamsVec( w );

	if( HasParam( ph, "seed" ) )
	{
		int seed;
		GetParam( ph, "seed", seed );
		_engine.seed( seed );
	}
	else
	{
		boost::random::random_device rng;
		_engine.seed( rng() );
	}

	double updateRate;
	GetParamRequired( ph, "update_rate", updateRate );
	_timer = ph.createTimer( ros::Duration( 1.0/updateRate ),
	                         &DiscretePolicyManager::UpdateCallback,
	                         this );
}

const DiscretePolicyManager::NetworkType&
DiscretePolicyManager::GetPolicyModule() const
{
	return *_network;
}

void DiscretePolicyManager::UpdateCallback( const ros::TimerEvent& event )
{
	StampedFeatures inputs;
	if( !_inputStreams.ReadStream( event.current_real, inputs ) )
	{
		ROS_WARN_STREAM( "Could not read input stream." );
		return;
	}

	// Generate probability mass function values
	_networkInput.SetOutput( inputs.features );
	_networkInput.Invalidate();
	_networkInput.Foreprop();
	VectorType pmf = _network->GetOutput();

	// Sample from PMF
	std::vector<double> weights( pmf.data(), pmf.data() + pmf.size() );
	std::vector<unsigned int> draws;
	NaiveWeightedSample( weights, 1, draws, _engine );
	unsigned int actionIndex = draws[0];

	// Convert single index into multiple indices
	std::vector<unsigned int> indices;
	indices = multibase_long_div( actionIndex, _policyInterface.GetNumSettings() );
	_policyInterface.SetOutputIndices( indices );

	// TODO name policy
	DiscreteParamAction action( event.current_real,
	                            inputs.features,
	                            actionIndex );
	_actionPub.publish( action.ToMsg() );
}

void DiscretePolicyManager::ParamCallback( const FloatVectorStamped::ConstPtr& msg )
{
	_networkParameters->SetParamsVec( GetVectorView( msg->values ) );
}

}