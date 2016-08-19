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
	

	unsigned int numHiddenLayers, layerWidth;
	GetParamRequired( ph, "network/type", _moduleType );
	GetParamRequired( ph, "network/num_hidden_layers", numHiddenLayers );
	GetParamRequired( ph, "network/layer_width", layerWidth );
	unsigned int inputDim = _inputStreams.GetDim();
	unsigned int matDim = _policyInterface.GetNumOutputs();

	if( _moduleType == "fixed_variance_gaussian" )
	{
		_network = std::make_shared<FixedVarianceGaussian>( inputDim, 
		                                                    matDim, 
		                                                    numHiddenLayers, 
		                                                    layerWidth );
	}
	else if( _moduleType == "variable_variance_gaussian" )
	{
		_network = std::make_shared<VariableVarianceGaussian>( inputDim, 
		                                                       matDim, 
		                                                       numHiddenLayers, 
		                                                       layerWidth );
	}
	else
	{
		throw std::invalid_argument( "Unknown policy type: " + _moduleType );
	}

	_network->SetInputSource( &_networkInput );
	_networkParameters = _network->CreateParameters();

	double initScale;
	GetParam( ph, "network/initial_magnitude", initScale, 1.0 );
	VectorType paramVec = _networkParameters->GetParamsVec();
	percepto::randomize_vector( paramVec, -initScale, initScale );
	_networkParameters->SetParamsVec( paramVec );

	// Read initialization if we have it
	const VectorType& lowerLimit = _policyInterface.GetLowerLimits();
	const VectorType& upperLimit = _policyInterface.GetUpperLimits();
	
	std::vector<std::string> paramNames = _policyInterface.GetParameterNames();
	_policyScales = Eigen::ArrayXd( matDim );
	_policyOffsets = VectorType( matDim );
	
	VectorType initMean( matDim );
	MatrixType initInfo = MatrixType::Identity( matDim, matDim );
	for( unsigned int i = 0; i < paramNames.size(); ++i )
	{
		GetParam( ph, 
		          "parameters/" + paramNames[i] + "/scale", 
		          _policyScales(i),
		          std::abs( upperLimit(i) - lowerLimit(i) ) );

		GetParam( ph, 
		          "parameters/" + paramNames[i] + "/offset", 
		          _policyOffsets(i),
		          lowerLimit(i) );

		GetParam( ph, 
		          "parameters/" + paramNames[i] + "/initial_mean", 
		          initMean(i),
		          0.5 * ( upperLimit(i) - lowerLimit(i) ) );
		initMean(i) = initMean(i) / _policyScales(i) - 0.5;

		double stdDev;
		GetParam( ph, 
		          "parameters/" + paramNames[i] + "/initial_std_dev", 
		          stdDev,
		          ( upperLimit(i) - lowerLimit(i) ) / 3.0 );
		double initVar = ( stdDev / _policyScales(i) ) * ( stdDev / _policyScales(i) );
		initInfo(i,i) = 1.0 / initVar;
	}

	_network->InitializeMean( initMean );
	_network->InitializeInformation( initInfo );
	ROS_INFO_STREAM( "Initializing to mean: " << initMean.transpose() << std::endl <<
	                 "info: " << std::endl << initInfo );
	ROS_INFO_STREAM( "Network initialized: " << std::endl << *_network );
}

ContinuousPolicyModule::Ptr
ContinuousPolicyManager::GetPolicyModule() const
{
	if( _moduleType == "fixed_variance_gaussian" )
	{
		FixedVarianceGaussian::Ptr net = std::dynamic_pointer_cast<FixedVarianceGaussian>( _network );
		return std::make_shared<FixedVarianceGaussian>( *net );
	}
	else if( _moduleType == "variable_variance_gaussian" )
	{
		VariableVarianceGaussian::Ptr net = std::dynamic_pointer_cast<VariableVarianceGaussian>( _network );
		return std::make_shared<VariableVarianceGaussian>( *net );
	}
	else
	{
		throw std::invalid_argument( "Unknown policy type: " + _moduleType );
	}
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

const Eigen::ArrayXd& ContinuousPolicyManager::GetScales() const
{
	return _policyScales;
}

const VectorType& ContinuousPolicyManager::GetOffsets() const
{
	return _policyOffsets;
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
	out.mean = _network->GetMeanSource().GetOutput();
	out.info = _network->GetInfoSource().GetOutput();
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