#include "covreg/InnovationProblem.h"

namespace argus
{

InnovationLikelihoodProblem::InnovationLikelihoodProblem( percepto::Parameters::Ptr params,
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
	if( episodes.empty() ) { return; }
	loss.RemoveOldestSource();
	episodes.pop_front();
}

size_t InnovationLikelihoodProblem::NumEpisodes() const
{
	return episodes.size(); 
}

KalmanFilterEpisode* 
InnovationLikelihoodProblem::GetOldestEpisode()
{
	if( episodes.empty() ) { return nullptr; }
	return &episodes.front();
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
	// clock_t start = clock();
	regularizer.Foreprop();

	loss.Resample();
	const std::vector<unsigned int>& activeEpsInds = loss.GetActiveInds();
	BOOST_FOREACH( const unsigned int& ind, activeEpsInds )
	{
		episodes[ind].Foreprop();
	}
	// clock_t finish = clock();
	// std::cout << "Foreprop took: " << ((double) finish - start)/CLOCKS_PER_SEC << std::endl;
}

void InnovationLikelihoodProblem::ForepropSame()
{
	// clock_t start = clock();
	regularizer.Foreprop();

	loss.UseSameSamples();
	const std::vector<unsigned int>& activeEpsInds = loss.GetActiveInds();
	BOOST_FOREACH( const unsigned int& ind, activeEpsInds )
	{
		episodes[ind].Foreprop();
	}
	// clock_t finish = clock();
	// std::cout << "Foreprop took: " << ((double) finish - start)/CLOCKS_PER_SEC << std::endl;
}
void InnovationLikelihoodProblem::ForepropAll()
{
	regularizer.Foreprop();
	BOOST_FOREACH( KalmanFilterEpisode& ep, episodes )
	{
		ep.ForepropAll();
	}
	loss.ParentCost::Foreprop();
}

void InnovationLikelihoodProblem::Backprop()
{
	// clock_t start = clock();
	objective.Backprop( MatrixType::Identity(1,1) );
	// clock_t finish = clock();
	// std::cout << "Problem backprop took: " << ((double) finish - start)/CLOCKS_PER_SEC << std::endl;
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