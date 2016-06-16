#pragma once

#include "covreg/ModuleDefinitions.h"
#include <list>

namespace argus
{

class KalmanFilterEpisode
{
public:
	// We use deques here so that growing the container does not invalidate
	// any self-pointers used the the predict/update modules
	
	// The estimate covariance at the beginning of the episode
	percepto::TerminalSource<MatrixType> initCov;

	// The predict steps in this episode
	std::deque<KalmanFilterPredictModule> predicts;

	// The update steps in this episode
	std::deque<KalmanFilterUpdateModule> updates;

	// This forces the avgInnoLL to always be able to output, even with 0 updates
	percepto::TerminalSource<double> offsetInnoLL;

	percepto::MeanCost<double> avgInnoLL;


	enum ClipType
	{
		CLIP_TYPE_NONE = 0,
		CLIP_TYPE_PREDICT,
		CLIP_TYPE_UPDATE
	};

	// Order of predicts/updates for printing
	std::vector<ClipType> order;

	ClipType tailType;

	KalmanFilterEpisode( const MatrixType& Sinit );

	size_t NumUpdates() const;

	template <class ...Args>
	void EmplacePredict( Args&&... args )
	{
		predicts.emplace_back( args... );
		tailType = CLIP_TYPE_PREDICT;
		order.push_back( CLIP_TYPE_PREDICT );
	}

	template <class ...Args>
	void EmplaceUpdate( Args&&... args )
	{
		updates.emplace_back( args... );
		avgInnoLL.AddSource( &updates.back().innovationLL );
		
		tailType = CLIP_TYPE_UPDATE;
		order.push_back( CLIP_TYPE_UPDATE );
	}

	percepto::Source<MatrixType>* GetTailSource();

	percepto::Source<double>* GetLL();

	// NOTE: We do not invalidate Sprev because we can't foreprop it
	void Invalidate();

	void Foreprop();

private:
	// Forbid copying
	KalmanFilterEpisode( const KalmanFilterEpisode& other );

	// Forbid assigning
	KalmanFilterEpisode& operator=( const KalmanFilterEpisode& other );

};

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