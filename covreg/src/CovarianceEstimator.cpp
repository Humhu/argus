#include "covreg/CovarianceEstimator.h"

#include "percepto/utils/Randomization.hpp"

#include "argus_utils/utils/MatrixUtils.h"

#define VAR_RAND_RANGE (1E-3)

using namespace covreg;

namespace argus
{

CovarianceEstimator::CovarianceEstimator( const std::string& source,
                                          unsigned int featDim,
                                          unsigned int matDim, 
                                          unsigned int numHiddenLayers, 
                                          unsigned int layerWidth )
: _sourceName( source ),
  _inDim( featDim ),
  _outDim( matDim ),
  _psd( featDim, matDim, numHiddenLayers, layerWidth )
{
	_psd.SetSource( &_psdPort );
	_lParams = _psd.lReg.CreateParameters();
	_dParams = _psd.dReg.CreateParameters();
	_params = std::make_shared<percepto::ParameterWrapper>();
	_params->AddParameters( _lParams );
	// _params->AddParameters( _dParams );
	for( unsigned int i = 0; i < _dParams.size(); i++ )
	{
		_params->AddParameters( _dParams[i] );
	}
}

CovarianceEstimator::CovarianceEstimator( const std::string& source, 
                                          const YAML::Node& info )
: _sourceName( source ),
  _inDim( info["input_dim"].as<unsigned int>() ),
  _outDim( info["output_dim"].as<unsigned int>() ),
  _psd( info["input_dim"].as<unsigned int>(),
        info["output_dim"].as<unsigned int>(),
        info["hidden_layers"].as<unsigned int>(),
        info["layer_width"].as<unsigned int>() )
{
	_psd.SetSource( &_psdPort );
	_lParams = _psd.lReg.CreateParameters();
	_dParams = _psd.dReg.CreateParameters();
	_params = std::make_shared<percepto::ParameterWrapper>();
	_params->AddParameters( _lParams );
	// _params->AddParameters( _dParams );
	for( unsigned int i = 0; i < _dParams.size(); i++ )
	{
		_params->AddParameters( _dParams[i] );
	}
}

unsigned int CovarianceEstimator::InputDim() const { return _inDim; }

unsigned int CovarianceEstimator::OutputDim() const { return _outDim; }

void CovarianceEstimator::RandomizeVarianceParams()
{
	for( unsigned int i = 0; i < _dParams.size(); i++ )
	{
		VectorType varParams( _dParams[i]->ParamDim() );
		percepto::randomize_vector( varParams, -VAR_RAND_RANGE, VAR_RAND_RANGE );
		_dParams[i]->SetParamsVec( varParams );
		// _dParams[i]->SetParamsVec( VectorType::Zero( _dParams[i]->ParamDim() ) );
	}
	// VectorType varParams = VectorType::Ones( _dParams->ParamDim() );
	// _dParams->SetParamsVec( varParams );
}

void CovarianceEstimator::ZeroCorrelationParams()
{
	VectorType corrParams( _lParams->ParamDim() );
	corrParams.setZero();
	_lParams->SetParamsVec( corrParams );
}

CovarianceEstimatorInfo CovarianceEstimator::GetParamsMsg() const
{
	CovarianceEstimatorInfo info;
	info.source_name = _sourceName;
	VectorType params = _params->GetParamsVec();
	info.parameters.resize( params.size() );
	GetVectorView( info.parameters ) = params;
	return info;
}

void CovarianceEstimator::SetParamsMsg( const CovarianceEstimatorInfo& info )
{
	if( info.source_name != _sourceName )
	{
		throw std::runtime_error( "CovarianceEstimator: Info source: " + info.source_name +
		                          " does not match source: " + _sourceName );
	}
	if( info.parameters.size() != _params->ParamDim() )
	{
		throw std::runtime_error( "CovarianceEstimator: Info param size: " + 
		                          std::to_string( info.parameters.size() ) +
		                          " does not match parameter dim: " +
		                          std::to_string( _params->ParamDim() ) );
	}
	_params->SetParamsVec( GetVectorView( info.parameters ) );
}

MatrixType CovarianceEstimator::Evaluate( const VectorType& input )
{
	_psdPort.SetOutput( input );
	_psdPort.Foreprop();
	return _psd.GetOutput();
}

percepto::Parameters::Ptr
CovarianceEstimator::GetParamSet() 
{ 
	return std::dynamic_pointer_cast<percepto::Parameters>( _params );
}

const VarReLUPosDefModule&
CovarianceEstimator::GetModule()
{
	return _psd;
}

// covreg::CovarianceEstimatorInfo
// CovarianceEstimator::GetInfoMessage() const
// {
// 	covreg::CovarianceEstimatorInfo info;
// 	info.source_name = sourceName;
// 	SerializeMatrix( _lReg.GetParamsVec(), info.correlation_parameters );
// 	info.variance_info.activation_type = "leaky_relu";
// 	info.variance_info.activation_params = std::vector<double>(2);
// 	info.variance_info.activation_params[0] = _dReg.GetActivation().activeSlope;
// 	info.variance_info.activation_params[1] = _dReg.GetActivation().inactiveSlope;
// 	info.variance_info.output_dim = _dReg.OutputDim();
// 	info.variance_info.input_dim = _dReg.InputDim();
// 	info.variance_info.num_hidden_layers = _dReg.NumHiddenLayers();
// 	SerializeMatrix( _dReg.GetParamsVec(), info.variance_info.parameters );
// 	info.offset = MatrixToMsg( _pdModule.GetOffset() );
// 	return info;
// }

}