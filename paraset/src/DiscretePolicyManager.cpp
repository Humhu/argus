#include "paraset/DiscretePolicyManager.h"
#include "argus_utils/random/RandomUtils.hpp"

#include "percepto/utils/Randomization.hpp"

#include <boost/random/random_device.hpp>

namespace argus
{

DiscretePolicyManager::DiscretePolicyManager() {}

void DiscretePolicyManager::Initialize( ros::NodeHandle& ph )
{
	_policyInterface.Initialize( ph );

	ros::NodeHandle subh( ph.resolveName("input_streams") );
	_inputStreams.Initialize( subh );

	unsigned int inputDim = _inputStreams.GetDim();
	unsigned int outputDim = _policyInterface.GetNumCombinations();
	unsigned int numHiddenLayers, layerWidth;
	GetParamRequired( ph, "network/num_hidden_layers", numHiddenLayers );
	GetParamRequired( ph, "network/layer_width", layerWidth );
	_network = std::make_shared<NormalizedPerceptron>( inputDim, 
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

void DiscretePolicyManager::UpdateCallback( const ros::TimerEvent& event )
{
	StampedFeatures inputs;
	if( !_inputStreams.ReadStream( event.current_real, inputs ) )
	{
		ROS_WARN_STREAM( "Could not read input stream." );
		return;
	}

	_networkInput.SetOutput( inputs.features );
	_networkInput.Invalidate();
	_networkInput.Foreprop();
	VectorType pmf = _network->GetOutput();
	ROS_INFO_STREAM( "pmf: " << pmf.transpose() << std::endl );

	std::vector<double> weights( pmf.data(), pmf.data() + pmf.size() );
	std::vector<unsigned int> draws;
	NaiveWeightedSample( weights, 1, draws, _engine );
	_policyInterface.SetOutput( draws[0] );
}

}