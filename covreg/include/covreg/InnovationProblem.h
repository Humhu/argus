#pragma once

#include "covreg/KalmanFilterEpisode.h"

#include <percepto/optim/StochasticMeanCost.hpp>
#include <percepto/optim/ParameterL2Cost.hpp>

#include <list>

namespace argus
{

std::ostream& operator<<( std::ostream& os, const KalmanFilterEpisode& episode );

struct InnovationLikelihoodProblem
{

	std::deque<KalmanFilterEpisode> episodes;
	percepto::StochasticMeanCost<double> loss;
	percepto::ParameterL2Cost regularizer;
	percepto::AdditiveWrapper<double> objective;

	InnovationLikelihoodProblem( percepto::Parameters* params,
	                             double l2Weight,
	                             unsigned int batchSize );

	template <class ...Args>
	void EmplaceEpisode( Args&& ...args )
	{
		episodes.emplace_back( args... );
		loss.AddSource( episodes.back().GetLL() );
	}

	void RemoveOldestEpisode();

	size_t NumEpisodes() const;

	KalmanFilterEpisode* GetCurrentEpisode();

	const KalmanFilterEpisode* GetCurrentEpisode() const;

	void Invalidate();

	void Foreprop();

	void ForepropAll();

	void Backprop();

	double GetOutput() const;

private:

	// Forbid copying
	InnovationLikelihoodProblem( const InnovationLikelihoodProblem& other );
	
	// Forbid assigning
	InnovationLikelihoodProblem& 
	operator=( const InnovationLikelihoodProblem& other );
};

std::ostream& operator<<( std::ostream& os, 
                          const InnovationLikelihoodProblem& problem );

}