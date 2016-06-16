#include "covreg/InnovationProblem.h"

namespace argus
{

InnovationLikelihoodProblem::InnovationLikelihoodProblem( percepto::Parameters* params,
                                                          double l2Weight,
                                                          unsigned int batchSize )
{
	loss.SetBatchSize( batchSize );
	regularizer.SetParameters( params );
	regularizer.SetWeight( l2Weight );
	objective.SetSourceA( &loss );
	objective.SetSourceB( &regularizer );
}

void InnovationLikelihoodProblem::RemoveOldestEpisode()
{
	loss.RemoveOldestSource();
	episodes.pop_front();
}

size_t InnovationLikelihoodProblem::NumEpisodes() const
{
	return episodes.size(); 
}

KalmanFilterEpisode* 
InnovationLikelihoodProblem::GetCurrentEpisode()
{
	if( episodes.empty() ) { return nullptr; }
	return &episodes.back();
}

const KalmanFilterEpisode* 
InnovationLikelihoodProblem::GetCurrentEpisode() const
{
	if( episodes.empty() ) { return nullptr; }
	return &episodes.back();
}

void InnovationLikelihoodProblem::Invalidate()
{
	regularizer.Invalidate();
	BOOST_FOREACH( KalmanFilterEpisode& ep, episodes )
	{
		ep.Invalidate();
	}
}

void InnovationLikelihoodProblem::Foreprop()
{
	regularizer.Foreprop();

	loss.Resample();
	const std::vector<unsigned int>& activeEpsInds = loss.GetActiveInds();
	BOOST_FOREACH( const unsigned int& ind, activeEpsInds )
	{
		episodes[ind].Foreprop();
	}
}

void InnovationLikelihoodProblem::ForepropAll()
{
	regularizer.Foreprop();
	BOOST_FOREACH( KalmanFilterEpisode& ep, episodes )
	{
		ep.Foreprop();
	}
	loss.ParentCost::Foreprop();
}

void InnovationLikelihoodProblem::Backprop()
{
	objective.Backprop( MatrixType() );
}

double InnovationLikelihoodProblem::GetOutput() const
{
	return objective.GetOutput();
}

std::ostream& operator<<( std::ostream& os, const InnovationLikelihoodProblem& problem )
{
	os << "Episodes: " << std::endl;
	for( unsigned int i = 0; i < problem.episodes.size(); i++ )
	{
		os << problem.episodes[i] << std::endl;
	}

	os << "regularizer penalty: " << problem.regularizer.GetOutput() << std::endl;
	os << "mean loss: " << problem.loss.ParentCost::GetOutput() << std::endl;
	os << "stochastic objective: " << problem.objective.GetOutput() << std::endl;
	return os;
}

}