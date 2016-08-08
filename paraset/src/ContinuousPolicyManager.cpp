#include "paraset/ContinuousPolicyManager.h"
#include "paraset/ContinuousParamAction.h"

#include "argus_utils/utils/MatrixUtils.h"

#include "percepto/utils/Randomization.hpp"

using namespace paraset;
using namespace argus_msgs;

namespace argus
{

ContinuousPolicyManager::ContinuousPolicyManager() {}

void ContinuousPolicyManager::Initialize( ros::NodeHandle& nh,
                                          ros::NodeHandle& ph )
{
	_policyInterface.Initialize( ph );

	_actionPub = ph.advertise<ContinuousParamAction>( "actions", 0 );
	_paramSub = nh.subscribe( "param_updates", 
	                          1, 
	                          &ContinuousPolicyManager::ParamCallback, 
	                          this );

	ros::NodeHandle subh( ph.resolveName("input_streams") );
	_inputStreams.Initialize( subh );

	unsigned int inputDim = _inputStreams.GetDim();
	unsigned int matDim = _policyInterface.GetNumOutputs();
	unsigned int numHiddenLayers, layerWidth;
	GetParamRequired( ph, "network/num_hidden_layers", numHiddenLayers );
	GetParamRequired( ph, "network/layer_width", layerWidth );
	_network = std::make_shared<VarReLUGaussian>( inputDim, 
	                                              matDim, 
	                                              numHiddenLayers, 
	                                              layerWidth );
	_network->SetSource( &_networkInput );
	_networkParameters = _network->CreateParameters();
	VectorType w = _networkParameters->GetParamsVec();
	percepto::randomize_vector( w, -0.1, 0.1 );
	_networkParameters->SetParamsVec( w );

	int seed;
	if( HasParam( ph, "seed" ) )
	{
		GetParam( ph, "seed", seed );
	}
	else
	{
		boost::random::random_device rng;
		seed = rng();
	}
	_dist = MultivariateGaussian<>( matDim, seed );

	GetParam( ph, "sample_devs_bound", _sampleDevs, 3.0 );

	double updateRate;
	GetParamRequired( ph, "update_rate", updateRate );
	_timer = ph.createTimer( ros::Duration( 1.0/updateRate ),
	                         &ContinuousPolicyManager::UpdateCallback,
	                         this );
}

void ContinuousPolicyManager::UpdateCallback( const ros::TimerEvent& event )
{
	StampedFeatures inputs;
	if( !_inputStreams.ReadStream( event.current_real, inputs ) )
	{
		ROS_WARN_STREAM( "Could not read input stream." );
		return;
	}

	// Generate mean and covariance
	_networkInput.SetOutput( inputs.features );
	_networkInput.Invalidate();
	_networkInput.Foreprop();

	// Sample from mean and covariance
	VectorType mean = _network->meanWrapper.GetOutput();
	MatrixType cov = _network->pdModule.GetOutput();
	// ROS_INFO_STREAM( "mean: " << mean.transpose() << std::endl <<
	//                  "cov: " << std::endl << cov );
	_dist.SetMean( mean );
	_dist.SetCovariance( cov );
	VectorType output = _dist.Sample( _sampleDevs );
	// ROS_INFO_STREAM( "sampled: " << output.transpose() );

	// Set output
	std::vector<double> outVec( output.data(), output.data() + output.size() );
	_policyInterface.SetOutput( outVec );

	// TODO Name policy
	ContinuousParamAction outMsg;
	outMsg.header.stamp = event.current_real;
	SerializeMatrix( inputs.features, outMsg.inputs );
	outMsg.outputs = outVec;
	_actionPub.publish( outMsg );
}

void ContinuousPolicyManager::ParamCallback( const FloatVectorStamped::ConstPtr& msg )
{
	_networkParameters->SetParamsVec( GetVectorView( msg->values ) );
}

}