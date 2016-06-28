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
  _psd( featDim, matDim, numHiddenLayers, layerWidth ),
  _learnCorrelations( false )
  // _psd( matDim )
{
	_psd.SetSource( &_psdPort );
	_lParams = _psd.lReg.CreateParameters();
	_dParams = _psd.dReg.CreateParameters();
	_params = std::make_shared<percepto::ParameterWrapper>();
	// _params->AddParameters( _lParams );
	_params->AddParameters( _dParams );
}

CovarianceEstimator::CovarianceEstimator( const std::string& source, 
                                          const YAML::Node& info )
: _sourceName( source ),
  _inDim( info["input_dim"].as<unsigned int>() ),
  _outDim( info["output_dim"].as<unsigned int>() ),
  _psd( info["input_dim"].as<unsigned int>(),
        info["output_dim"].as<unsigned int>(),
        info["hidden_layers"].as<unsigned int>(),
        info["layer_width"].as<unsigned int>() ),
  _learnCorrelations( info["learn_correlations"].as<bool>() )
  // _psd( info["output_dim"].as<unsigned int>() )
{
	_psd.SetSource( &_psdPort );
	_lParams = _psd.lReg.CreateParameters();
	_dParams = _psd.dReg.CreateParameters();
	_params = std::make_shared<percepto::ParameterWrapper>();
	_params->AddParameters( _dParams );
	if( _learnCorrelations )
	{
		_params->AddParameters( _lParams );
	}
}

void CovarianceEstimator::EnableCorrelationLearning()
{
	if( !_learnCorrelations )
	{
		_learnCorrelations = true;
		_params->AddParameters( _lParams );
	}
}

unsigned int CovarianceEstimator::InputDim() const { return _inDim; }

unsigned int CovarianceEstimator::OutputDim() const { return _outDim; }

void CovarianceEstimator::RandomizeVarianceParams()
{
	VectorType varParams( _dParams->ParamDim() );
	percepto::randomize_vector( varParams, -VAR_RAND_RANGE, VAR_RAND_RANGE );
	_dParams->SetParamsVec( varParams );
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
	// GetVectorView( info.parameters ) = params;
	SerializeMatrix( params, info.parameters );
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
	_psd.Invalidate();
	_psdPort.Foreprop();
	MatrixType out = _psd.GetOutput();
	return out;
}

percepto::Parameters::Ptr
CovarianceEstimator::GetParamSet() 
{ 
	return std::dynamic_pointer_cast<percepto::Parameters>( _params );
}

const CovarianceEstimator::ModuleType& CovarianceEstimator::GetModule()
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