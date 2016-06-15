#include "covreg/CovarianceEstimator.h"
#include "percepto/utils/Randomization.hpp"
#include "argus_utils/utils/MatrixUtils.h"

namespace argus
{

CovarianceEstimator::CovarianceEstimator( const std::string& source,
                                          unsigned int featDim,
                                          unsigned int matDim, 
                                          unsigned int numHiddenLayers, 
                                          unsigned int layerWidth )
: _psd( featDim, matDim, numHiddenLayers, layerWidth )
{
	_lParams = _psd.lReg.CreateParameters();
	_dParams = _psd.dReg.CreateParameters();
	_params = std::make_shared<percepto::ParameterWrapper>();
	_params->AddParameters( _lParams );
	for( unsigned int i = 0; i < _dParams.size(); i++ )
	{
		_params->AddParameters( _dParams[i] );
	}
}

void CovarianceEstimator::RandomizeVarianceParams()
{
	for( unsigned int i = 0; i < _dParams.size(); i++ )
	{
		VectorType varParams( _dParams[i]->ParamDim() );
		percepto::randomize_vector( varParams );
		_dParams[i]->SetParamsVec( varParams );
	}
}

void CovarianceEstimator::ZeroCorrelationParams()
{
	VectorType corrParams( _lParams->ParamDim() );
	corrParams.setZero();
	_lParams->SetParamsVec( corrParams );
}

MatrixType CovarianceEstimator::Evaluate( const VectorType& input )
{
	return _psd.Evaluate( input );
}

percepto::Parameters::Ptr
CovarianceEstimator::GetParamSet() 
{ 
	return std::dynamic_pointer_cast<percepto::Parameters>( _params );
}

const PositiveDefiniteModule&
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