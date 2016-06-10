#include "covreg/CovarianceEstimator.h"
#include "percepto/utils/Randomization.hpp"
#include "argus_utils/utils/MatrixUtils.h"

namespace argus
{

// TODO Use different activations from the msg
CovarianceEstimator::CovarianceEstimator( const covreg::CovarianceEstimatorInfo& info )
: sourceName( info.source_name ),
_cReg( info.correlation_parameters.size(), 1 ),
_vBaseReg( info.variance_info.input_dim, 
           info.variance_info.output_dim,
           info.variance_info.num_hidden_layers,
           info.variance_info.layer_width,
           percepto::HingeActivation( info.variance_info.activation_params[0],
                                      info.variance_info.activation_params[1] ) ),
_vBaseMod( _vBaseReg ),
_vMod( _vBaseMod ),
_psdMod( _cReg, _vMod ),
_pdMod( _psdMod, MsgToMatrix( info.offset ) ),
_pdReg( _vBaseMod, _pdMod ) 
{
	if( info.variance_info.activation_type != "leaky_relu" )
	{
		throw std::runtime_error( "CovarianceEstimator: Only leaky_relu activation is currently supported." );
	}
	_cReg.SetParamsVec( GetVectorView( info.correlation_parameters ) );
	_vBaseReg.SetParamsVec( GetVectorView( info.variance_info.parameters ) );
	_paramWrapper.AddParametric( &_cReg );
	_paramWrapper.AddParametric( &_vBaseReg );
}

CovarianceEstimator::CovarianceEstimator( const std::string& source,
                                          unsigned int matDim, 
                                          unsigned int featDim,
                                          unsigned int numHiddenLayers, 
                                          unsigned int layerWidth )
: sourceName( source ),
_cReg( matDim*(matDim-1)/2, 1 ),
_vBaseReg( featDim, matDim, numHiddenLayers, layerWidth,
            percepto::HingeActivation( 1.0, 1E-3 ) ),
_vBaseMod( _vBaseReg ),
_vMod( _vBaseMod ),
_psdMod( _cReg, _vMod ),
_pdMod( _psdMod, 1E-9 * MatrixType::Identity( matDim, matDim ) ),
_pdReg( _vBaseMod, _pdMod ) 
{
	_paramWrapper.AddParametric( &_cReg );
	_paramWrapper.AddParametric( &_vBaseReg );
}

void CovarianceEstimator::RandomizeVarianceParams()
{
	VectorType varParams = _vBaseReg.GetParamsVec();
	percepto::randomize_vector( varParams );
	_vBaseReg.SetParamsVec( varParams );
}

void CovarianceEstimator::ZeroCorrelationParams()
{
	VectorType corrParams = _cReg.GetParamsVec();
	corrParams.setZero();
	_cReg.SetParamsVec( corrParams );
}

void CovarianceEstimator::SetParameters( const VectorType& params )
{
	_paramWrapper.SetParamsVec( params );
}

VectorType CovarianceEstimator::GetParameters() const
{
	return _paramWrapper.GetParamsVec();
}

MatrixType CovarianceEstimator::Evaluate( const VectorType& input )
{
	return _pdReg.Evaluate( input );
}

covreg::CovarianceEstimatorInfo
CovarianceEstimator::GetInfoMessage() const
{
	covreg::CovarianceEstimatorInfo info;
	info.source_name = sourceName;
	SerializeMatrix( _cReg.GetParamsVec(), info.correlation_parameters );
	info.variance_info.activation_type = "leaky_relu";
	info.variance_info.activation_params = std::vector<double>(2);
	info.variance_info.activation_params[0] = _vBaseReg.GetActivation().activeSlope;
	info.variance_info.activation_params[1] = _vBaseReg.GetActivation().inactiveSlope;
	info.variance_info.output_dim = _vBaseReg.OutputDim();
	info.variance_info.input_dim = _vBaseReg.InputDim();
	info.variance_info.num_hidden_layers = _vBaseReg.NumHiddenLayers();
	SerializeMatrix( _vBaseReg.GetParamsVec(), info.variance_info.parameters );
	info.offset = MatrixToMsg( _pdMod.offset );
	return info;
}

}