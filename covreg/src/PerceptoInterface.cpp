#include "covreg/PerceptoInterface.h"
#include "percepto/utils/Randomization.hpp"
#include <iostream>

namespace argus
{

MatrixRegressor::MatrixRegressor( unsigned int matDim, unsigned int featDim,
                                  unsigned int numHiddenLayers, 
                                  unsigned int layerWidth )
: _vBaseReg( featDim, matDim, numHiddenLayers, layerWidth,
            percepto::HingeActivation( 1.0, 1E-3 ) ),
_vReg( _vBaseReg ),
_cReg( matDim*(matDim-1)/2, 1 ),
_psdReg( _cReg, _vReg ),
_pdReg( _psdReg, 1E-9 * MatrixType::Identity( matDim, matDim ) ) {}

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
	_pdReg.SetParamsVec( params );
}

VectorType MatrixRegressor::GetParameters() const
{
	return _pdReg.GetParamsVec();
}

MatrixType MatrixRegressor::Evaluate( const VectorType& input )
{
	PDRegressor::InputType regIn;
	regIn.dInput = input;
	return _pdReg.Evaluate( regIn );
}

InnovationClipOptimizer::InnovationClipOptimizer( MatrixRegressor& qReg,
                                                  const InnovationClipParameters& params )
: _transReg( qReg.Regressor() ), _sill( _innoLLs, params.batchSize ), 
_mill( _innoLLs ),
_psill( _sill, params.l2Weight ), _maxClipLength( params.maxClipLength ),
_maxNumClips( params.numClipsToKeep ) {}

void InnovationClipOptimizer::AddObservationReg( MatrixRegressor& reg,
                                                 const std::string& name )
{
	_obsRegs.emplace( name, reg.Regressor() );
}

void InnovationClipOptimizer::AddPredict( const PredictInfo& info,
                                          const VectorType& input )
{
	_predictBuffer.emplace_front( info, input );
	if( _predictBuffer.size() > _maxClipLength )
	{
		_predictBuffer.pop_back();
	}
}

void InnovationClipOptimizer::AddUpdate( const UpdateInfo& info,
                                         const VectorType& input,
                                         const std::string& name )
{
	_clips.emplace_back( _transReg, _obsRegs.at( name ), info,
	                     input, _predictBuffer );
	_predictBuffer.clear();
	_innoLLs.emplace_back( *_clips.back().estV, info.innovation );
}

size_t InnovationClipOptimizer::NumClips() const { return _clips.size(); }

void InnovationClipOptimizer::Optimize( const percepto::SimpleConvergenceCriteria& criteria )
{
	percepto::AdamStepper stepper;
	percepto::SimpleConvergence convergence( criteria );
	percepto::AdamOptimizer optimizer( stepper, convergence );
	std::cout << "Initial cost: " << _mill.Evaluate() << std::endl;
	optimizer.Optimize( _psill );
	std::cout << "Final cost: " << _mill.Evaluate() << std::endl; 
}

InnovationClipOptimizer::ClipData::ClipData( RegressorType& qReg,
                                             RegressorType& rReg,
                                             const UpdateInfo& info, 
                                             const VectorType& in,
                                             std::deque<PredictData>& buff )
: estR( rReg, RegressorType::InputType( VectorType(), in ) ), 
sumTransQs( transQs ), sumTransQsR( sumTransQs, estR )
{
	estQs.reserve( buff.size() );
	transQs.reserve( buff.size() );
	MatrixType Facc = info.H;
	for( unsigned int i = 0; i < buff.size(); i++ )
	{
		const PredictInfo& predInfo = buff[i].first;
		const VectorType& qIn = buff[i].second;
		estQs.emplace_back( qReg, RegressorType::InputType( VectorType(), qIn ) );
		transQs.emplace_back( estQs[i], Facc );
		Facc = Facc * predInfo.F;
	}
	Facc = Facc * buff.back().first.Spre * Facc.transpose();
	estV = std::make_shared<InnovationCov>( sumTransQsR, Facc );
}

}