#include "paraset/ContinuousPolicyManager.h"
#include "paraset/ContinuousParamAction.h"

#include "argus_utils/utils/MatrixUtils.h"

#include "percepto/utils/Randomization.hpp"
#include <boost/foreach.hpp>
#include <sstream>

using namespace argus_msgs;

namespace argus
{

ContinuousPolicyManager::ContinuousPolicyManager() {}

void ContinuousPolicyManager::Initialize( ros::NodeHandle& nh,
                                          ros::NodeHandle& ph )
{
	_policyInterface.Initialize( nh, ph );

	// Initialize input listeners
	ros::NodeHandle subh( ph.resolveName("input_streams") );
	_inputStreams.Initialize( subh );

	std::string updateTopic;
	if( GetParam( ph, "update_topic", updateTopic ) )
	{
		_paramSub = nh.subscribe( updateTopic, 
		                          1, 
		                          &ContinuousPolicyManager::ParamCallback, 
		                          this );
	}
	
	unsigned int inputDim = _inputStreams.GetDim();
	unsigned int matDim = _policyInterface.GetNumOutputs();
	unsigned int numHiddenLayers, layerWidth;
	GetParamRequired( ph, "network/num_hidden_layers", numHiddenLayers );
	GetParamRequired( ph, "network/layer_width", layerWidth );
	_network = std::make_shared<NetworkType>( inputDim, 
	                                          matDim, 
	                                          numHiddenLayers, 
	                                          layerWidth );
	_network->reg.SetSource( &_networkInput );
	_networkParameters = _network->CreateParameters();
	VectorType w = _networkParameters->GetParamsVec();
	percepto::randomize_vector( w, -0.01, 0.01 );
	_networkParameters->SetParamsVec( w );

	// Read initialization if we have it
	std::vector<std::string> paramNames = _policyInterface.GetParameterNames();
	VectorType initMean = VectorType::Zero( paramNames.size() );
	Eigen::ArrayXd initStdDev = VectorType::Constant( paramNames.size(), 1.0 );
	for( unsigned int i = 0; i < paramNames.size(); ++i )
	{
		GetParam( ph, "parameters/" + paramNames[i] + "/initial_mean", initMean(i) );
		GetParam( ph, "parameters/" + paramNames[i] + "/initial_std_dev", initStdDev(i) );
	}

	VectorType initVariances = -( ( initStdDev * initStdDev ).log().matrix() );
	_network->SetOutputOffsets( initMean, initVariances );
}

const ContinuousPolicyManager::NetworkType&
ContinuousPolicyManager::GetPolicyModule() const
{
	return *_network;
}

StampedFeatures ContinuousPolicyManager::GetInput( const ros::Time& time )
{
	StampedFeatures input;
	if( !_inputStreams.ReadStream( time, input ) )
	{
		std::stringstream ss;
		ss << "Could not read input streams at time: " << time;
		throw std::out_of_range( ss.str() );
	}
	return input;
}

percepto::Parameters::Ptr ContinuousPolicyManager::GetParameters()
{
	return _networkParameters;
}

ContinuousPolicyManager::DistributionParameters 
ContinuousPolicyManager::GetDistributionParams( const ros::Time& time )
{
	StampedFeatures input = GetInput( time );

	// Generate mean and covariance
	_networkInput.SetOutput( input.features );
	_networkInput.Invalidate();
	_network->Invalidate();
	_networkInput.Foreprop();
	_network->Foreprop(); // TODO Clunky!

	DistributionParameters out;
	out.mean = _network->meanWrapper.GetOutput();
	out.info = _network->pdModule.GetOutput();
	return out;
}

void ContinuousPolicyManager::SetOutput( const VectorType& out )
{
	_policyInterface.SetOutput( out );
}

unsigned int ContinuousPolicyManager::GetNumOutputs() const
{
	return _policyInterface.GetNumOutputs();
}

ContinuousPolicy& ContinuousPolicyManager::GetPolicyInterface()
{
	return _policyInterface;
}

void ContinuousPolicyManager::ParamCallback( const FloatVectorStamped::ConstPtr& msg )
{
	_networkParameters->SetParamsVec( GetVectorView( msg->values ) );
}

}