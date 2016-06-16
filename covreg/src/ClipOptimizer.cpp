#include "covreg/ClipOptimizer.h"

#include <iostream>
#include <sstream>

using namespace percepto;

namespace argus
{

InnovationClipOptimizer::InnovationClipOptimizer( CovarianceEstimator& qReg,
                                                  const InnovationClipParameters& params )
: _transReg( qReg ), 
_problem( &_paramWrapper, params.l2Weight, params.batchSize ),
_maxNumEpisodes( params.numEpisodesToKeep ),
_maxEpisodeLength( params.maxEpisodeLength )
{
	_paramWrapper.AddParameters( qReg.GetParamSet() );
}

void InnovationClipOptimizer::AddObservationReg( CovarianceEstimator& reg,
                                                 const std::string& name )
{
	WriteLock lock( _mutex );
	_obsRegs.emplace( name, reg );
	_paramWrapper.AddParameters( reg.GetParamSet() );
}

void InnovationClipOptimizer::AddPredict( const PredictInfo& info,
                                          const VectorType& input )
{
	WriteLock lock( _mutex );

	if( _problem.NumEpisodes() > _maxNumEpisodes )
	{
		_problem.RemoveOldestEpisode();
	}

	KalmanFilterEpisode* currentEpisode = _problem.GetCurrentEpisode();
	if( currentEpisode == nullptr ||
	    currentEpisode->NumUpdates() >= _maxEpisodeLength )
	{
		_problem.EmplaceEpisode( info.Spre );
		currentEpisode = _problem.GetCurrentEpisode();
	}

	currentEpisode->EmplacePredict( currentEpisode->GetTailSource(), 
	                                _transReg.GetModule(),
	                                input,
	                                info.F );
}

bool InnovationClipOptimizer::AddUpdate( const UpdateInfo& info,
                                         const VectorType& input,
                                         const std::string& name )
{
	WriteLock lock( _mutex );

	if( _obsRegs.count( name ) == 0 ) 
	{
		throw std::runtime_error( "Received unregistered update." );
	}
	CovarianceEstimator& est = _obsRegs.at( name );
	
	if( _problem.NumEpisodes() > _maxNumEpisodes )
	{
		_problem.RemoveOldestEpisode();
	}

	KalmanFilterEpisode* currentEpisode = _problem.GetCurrentEpisode();
	if( currentEpisode == nullptr ||
	    currentEpisode->NumUpdates() >= _maxEpisodeLength )
	{
		_problem.EmplaceEpisode( info.Spre );
		currentEpisode = _problem.GetCurrentEpisode();
	}

	currentEpisode->EmplaceUpdate( currentEpisode->GetTailSource(), 
	                               est.GetModule(),
	                               input,
	                               info.H,
	                               info.innovation );
	return true;
}

size_t InnovationClipOptimizer::NumEpisodes() const
{
	return _problem.NumEpisodes();
}

size_t InnovationClipOptimizer::CurrentEpisodeLength() const
{
	if( NumEpisodes() == 0 ) { return 0; }
	return _problem.GetCurrentEpisode()->NumUpdates();
}

void InnovationClipOptimizer::InitializeOptimization( const percepto::SimpleConvergenceCriteria& criteria )
{
	WriteLock lock( _mutex );
	percepto::AdamParameters aparams;

	_stepper = std::make_shared<percepto::AdamStepper>( aparams );
	_convergence = std::make_shared<percepto::SimpleConvergence>( criteria );
	_optimizer = std::make_shared<percepto::AdamOptimizer>( *_stepper, 
	                                                        *_convergence,
	                                                        _paramWrapper );
}

void InnovationClipOptimizer::Optimize()
{	
	WriteLock lock( _mutex );
	_convergence->Reset();
	_optimizer->Optimize( _problem );
}

double InnovationClipOptimizer::CalculateCost()
{
	WriteLock lock( _mutex );
	_problem.Invalidate();
	_problem.ForepropAll();
	return _problem.loss.ParentCost::GetOutput();
}

void InnovationClipOptimizer::Print( std::ostream& os )
{
	WriteLock lock( _mutex );
	_problem.Invalidate();
	_problem.ForepropAll();
	os << "Optimization problem: " << std::endl << _problem;
}

std::ostream& operator<<( std::ostream& os, InnovationClipOptimizer& opt )
{
	opt.Print( os );
	return os;
}

}