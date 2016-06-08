#include "covreg/CovarianceEstimator.h"
#include "percepto/utils/Randomization.hpp"

namespace argus
{

MatrixRegressor::MatrixRegressor( unsigned int matDim, unsigned int featDim,
                                  unsigned int numHiddenLayers, 
                                  unsigned int layerWidth )
:_cReg( matDim*(matDim-1)/2, 1 ),
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

void MatrixRegressor::RandomizeVarianceParams()
{
	VectorType varParams = _vBaseReg.GetParamsVec();
	percepto::randomize_vector( varParams );
	_vBaseReg.SetParamsVec( varParams );
}

void MatrixRegressor::ZeroCorrelationParams()
{
	VectorType corrParams = _cReg.GetParamsVec();
	corrParams.setZero();
	_cReg.SetParamsVec( corrParams );
}

void MatrixRegressor::SetParameters( const VectorType& params )
{
	_paramWrapper.SetParamsVec( params );
}

VectorType MatrixRegressor::GetParameters() const
{
	return _paramWrapper.GetParamsVec();
}

MatrixType MatrixRegressor::Evaluate( const VectorType& input )
{
	return _pdReg.Evaluate( input );
}

}