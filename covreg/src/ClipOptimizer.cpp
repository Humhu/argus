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

	_predBuffer.emplace_back( info, input );
}

bool InnovationClipOptimizer::AddUpdate( const UpdateInfo& info,
                                         const VectorType& input,
                                         const std::string& name,
                                         double scale )
{
	WriteLock lock( _mutex );

	if( _obsRegs.count( name ) == 0 ) 
	{
		throw std::runtime_error( "Received unregistered update." );
	}
	CovarianceEstimator& est = _obsRegs.at( name );

	if( _predBuffer.size() == 0 )
	{
		std::cout << "Received update with no predicts!" << std::endl;
		return false;
	}

	if( _problem.NumEpisodes() > _maxNumEpisodes )
	{
		_problem.RemoveOldestEpisode();
	}

	KalmanFilterEpisode* currentEpisode = _problem.GetCurrentEpisode();
	if( currentEpisode == nullptr ||
	    currentEpisode->NumUpdates() >= _maxEpisodeLength )
	{
		std::pair<PredictInfo,VectorType>& item = _predBuffer.front();
		PredictInfo& info = item.first;
		_problem.EmplaceEpisode( info.xpre, info.Spre );
		currentEpisode = _problem.GetCurrentEpisode();
	}

	for( unsigned int i = 0; i < _predBuffer.size(); i++ )
	{
		std::pair<PredictInfo,VectorType>& item = _predBuffer[i];
		PredictInfo& info = item.first;
		VectorType& input = item.second;
		currentEpisode->EmplacePredict( currentEpisode->GetTailState(),
		                                currentEpisode->GetTailCov(), 
		                                _transReg.GetModule(),
		                                info.dt,
		                                input,
		                                info.F );
	}
	_predBuffer.clear();

	currentEpisode->EmplaceUpdate( name,
	                               scale,
	                               currentEpisode->GetTailState(),
	                               currentEpisode->GetTailCov(), 
	                               est.GetModule(),
	                               input,
	                               info.observation,
	                               info.H );
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

void InnovationClipOptimizer::InitializeOptimization( const percepto::SimpleConvergenceCriteria& criteria,
                                                      const percepto::AdamParameters& params )
{
	WriteLock lock( _mutex );
	_stepper = std::make_shared<percepto::AdamStepper>( params );
	_convergence = std::make_shared<percepto::SimpleConvergence>( criteria );
	_optimizer = std::make_shared<percepto::AdamOptimizer>( *_stepper, 
	                                                        *_convergence,
	                                                        _paramWrapper );
}

bool InnovationClipOptimizer::Optimize()
{	
	WriteLock lock( _mutex );
	// _convergence->Reset();
	// percepto::OptimizationResults results = _optimizer->Optimize( _problem );
	// return results.converged;
	return _optimizer->StepOnce( _problem );
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
	_problem.Backprop();
	os << "Optimization problem: " << std::endl << _problem;
}

std::ostream& operator<<( std::ostream& os, InnovationClipOptimizer& opt )
{
	opt.Print( os );
	return os;
}

}