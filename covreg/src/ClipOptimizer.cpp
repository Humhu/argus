#include "covreg/ClipOptimizer.h"
#include "percepto/utils/Randomization.hpp"

#include <iostream>
#include <sstream>

namespace argus
{

InnovationClipOptimizer::InnovationClipOptimizer( MatrixRegressor& qReg,
                                                  const InnovationClipParameters& params )
: _transReg( qReg._pdReg ), _sill( _innoLLs, params.batchSize ), 
_mill( _innoLLs ),
_l2Cost( _paramWrapper, params.l2Weight ),
_psill( _sill, _l2Cost ), 
_maxClipLength( params.maxClipLength ),
_maxNumClips( params.numClipsToKeep ) 
{
	_paramWrapper.AddParametric( &qReg._paramWrapper );
}

void InnovationClipOptimizer::AddObservationReg( MatrixRegressor& reg,
                                                 const std::string& name )
{
	_obsRegs.emplace( name, reg._pdReg );
	_paramWrapper.AddParametric( &reg._paramWrapper );
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

bool InnovationClipOptimizer::AddUpdate( const UpdateInfo& info,
                                         const VectorType& input,
                                         const std::string& name )
{
	if( _obsRegs.count( name ) == 0 || _predictBuffer.empty() )
	{
		return false;
	}
	_clips.emplace_back( _transReg, _obsRegs.at( name ), info,
	                     input, _predictBuffer );
	_clips.back().sourceName = name;
	_clips.back().innovation = info.innovation;

	_predictBuffer.clear();
	_innoLLs.emplace_back( *_clips.back().estV, info.innovation );

	if( _clips.size() > _maxNumClips )
	{
		_clips.pop_front();
		_innoLLs.pop_front();
	}

	return true;
}

size_t InnovationClipOptimizer::NumClips() const { return _clips.size(); }

void InnovationClipOptimizer::InitializeOptimization( const percepto::SimpleConvergenceCriteria& criteria )
{
	percepto::AdamParameters aparams;
	// aparams.beta1 = 1.0 - 1E-6;
	// aparams.beta2 = 1.0 - 1E-4;
	_stepper = std::make_shared<percepto::AdamStepper>( aparams );
	_convergence = std::make_shared<percepto::SimpleConvergence>( criteria );
	_optimizer = std::make_shared<percepto::AdamOptimizer>( *_stepper, 
	                                                        *_convergence,
	                                                        _paramWrapper );
}

bool InnovationClipOptimizer::StepOnce()
{
	std::cout << "Initial mean cost: " << _mill.Evaluate() << std::endl;
	bool ret = _optimizer->StepOnce( _psill );
	std::cout << "Post mean cost: " << _mill.Evaluate() << std::endl;
	return ret;
}

void InnovationClipOptimizer::Optimize()
{
	_convergence->Reset();
	_optimizer->Optimize( _psill );
}

double InnovationClipOptimizer::GetCost() const
{
	return _mill.Evaluate();
}

void InnovationClipOptimizer::Print( unsigned int num ) const
{
	if( num > _clips.size() ) { num = _clips.size(); }
	for( unsigned int i = 0; i < num; i++ )
	{
		_clips[i].Print();
	}
}

InnovationClipOptimizer::ClipData::ClipData( RegressorType& qReg,
                                             RegressorType& rReg,
                                             const UpdateInfo& info, 
                                             const VectorType& in,
                                             std::deque<PredictData>& buff )
: estR( rReg, in ), 
sumTransQs( transQs ), 
sumTransQsR( sumTransQs, estR )
{
	if( buff.empty() )
	{
		throw std::runtime_error( "Cannot build clip without predicts." );
	}

	estQs.reserve( buff.size() );
	transQs.reserve( buff.size() );
	MatrixType Facc = info.H;
	for( unsigned int i = 0; i < buff.size(); i++ )
	{
		const PredictInfo& predInfo = buff[i].first;
		const VectorType& qIn = buff[i].second;
		estQs.emplace_back( qReg, qIn );
		transQs.emplace_back( estQs[i], Facc );
		Facc = Facc * predInfo.F;
	}

	MatrixType Sinit = buff.back().first.Spre;
	estV = std::make_shared<AugmentedInnoCov>( sumTransQsR, 
	                                           Facc * Sinit * Facc.transpose() );

	// std::cout << "estR: " << std::endl << estR.Evaluate() << std::endl;
	// std::cout << "estQs[0]: " << std::endl << estQs[0].Evaluate() << std::endl;
	// std::cout << "transQ[0]: " << std::endl << transQs[0].Evaluate() << std::endl;
	// std::cout << "sumTransQs: " << std::endl << sumTransQs.Evaluate() << std::endl;
	// std::cout << "sumTransQsR: " << std::endl << sumTransQsR.Evaluate() << std::endl;
	// std::cout << "estV: " << std::endl << estV->Evaluate() << std::endl;
}

void InnovationClipOptimizer::ClipData::Print() const
{
	std::stringstream ss;
	ss << "Clip source: " << sourceName << std::endl
	   << "\tinno: " << innovation.transpose() << std::endl
	   << "\tR features: " << estR.GetInput().transpose() << std::endl
	   << "\tR est: " << std::endl << estR.Evaluate() << std::endl
	   << "\t" << std::endl;
	for( unsigned int i = 0; i < estQs.size(); i++ )
	{
		ss << "\tQ features: " << estQs[i].GetInput().transpose() << std::endl
		   << "\tQ est: " << std::endl << estQs[i].Evaluate() << std::endl;
	}
	std::cout << ss.str() << std::endl;
}

}