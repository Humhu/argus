#pragma once

#include <percepto/optim/StochasticSumCost.hpp>
#include <percepto/optim/SumCost.hpp>

#include "covreg/KalmanFilterModules.h"

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

	// This forces the sumInnoLL to always be able to output, even with 0 updates
	percepto::TerminalSource<double> offsetInnoLL;

	// Constrained sums for each source
	std::deque<percepto::StochasticSumCost<double>> sourceSums;
	std::map<std::string, unsigned int> sourceInds;

	percepto::SumCost<double> sumInnoLL;

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
		// std::string postfix = std::to_string( predicts.size() - 1 );
		// predicts.back().qInput.name = "qInput " + postfix;
		// predicts.back().Q.name = "Q " + postfix;
		// predicts.back().Q.dReg.GetOutputSource().name = "Q dReg " + postfix;
		// predicts.back().Q.lReg.name = "Q lReg " + postfix;
	}

	template <class ...Args>
	void EmplaceUpdate( const std::string& name, 
	                    unsigned int maxSamples,
	                    Args&&... args )
	{
		updates.emplace_back( args... );
		updates.back().sourceName = name;
		updates.back().innovationLL.name = name;

		if( sourceInds.count( name ) == 0 )
		{
			sourceInds[name] = sourceSums.size();
			sourceSums.emplace_back();
			//sourceSums.back().SetSubsample( ssRate );
			sourceSums.back().SetBatchSize( maxSamples );
			sumInnoLL.AddSource( &sourceSums.back() );
		}
		
		sourceSums[sourceInds[name]].AddSource( &updates.back().innovationLL );
		// sumInnoLL.AddSource( &updates.back().innovationLL );

		tailType = CLIP_TYPE_UPDATE;
		order.push_back( CLIP_TYPE_UPDATE );
		std::string postfix = std::to_string( updates.size() - 1 );
		// updates.back().rInput.name = "rInput " + postfix;
		// updates.back().R.name = "R " + postfix;
		// updates.back().R.dReg.GetOutputSource().name = "R dReg " + postfix;
		// updates.back().R.lReg.name = "R lReg " + postfix;
	}

	percepto::Source<MatrixType>* GetTailSource();

	percepto::Source<double>* GetLL();

	// NOTE: We do not invalidate Sprev because we can't foreprop it
	void Invalidate();

	void Foreprop();

	void ForepropAll();

private:
	// Forbid copying
	KalmanFilterEpisode( const KalmanFilterEpisode& other );

	// Forbid assigning
	KalmanFilterEpisode& operator=( const KalmanFilterEpisode& other );

};

}