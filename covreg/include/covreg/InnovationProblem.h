#pragma once

#include "covreg/ModuleDefinitions.h"

namespace argus
{

class KalmanFilterEpisode
// TODO: Use this with stochastic mean cost by inherting from
//: percepto::Source<double>
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

	percepto::MeanCost<double> avgInnoLL;


	enum ClipType
	{
		CLIP_TYPE_NONE = 0,
		CLIP_TYPE_PREDICT,
		CLIP_TYPE_UPDATE
	};

	// Order of predicts/updates for printing
	std::vector<ClipType> order;

	ClipType rootType;
	ClipType tailType;

	KalmanFilterEpisode( const MatrixType& Sinit )
	: rootType( CLIP_TYPE_NONE ), 
	  tailType( CLIP_TYPE_NONE ) 
	{
		initCov.SetOutput( Sinit );
	}

	size_t NumUpdates() const { return updates.size(); }

	template <class ...Args>
	void EmplacePredict( Args&&... args )
	{
		predicts.emplace_back( args... );
		if( rootType == CLIP_TYPE_NONE )
		{
			rootType = CLIP_TYPE_PREDICT;
		}
		tailType = CLIP_TYPE_PREDICT;
		order.push_back( CLIP_TYPE_PREDICT );
	}

	template <class ...Args>
	void EmplaceUpdate( Args&&... args )
	{
		updates.emplace_back( args... );
		avgInnoLL.AddSource( &updates.back().innovationLL );
		if( rootType == CLIP_TYPE_NONE )
		{
			rootType = CLIP_TYPE_UPDATE;
		}
		tailType = CLIP_TYPE_UPDATE;
		order.push_back( CLIP_TYPE_UPDATE );
	}

	// Reassigns the source root
	void SetRootSource( percepto::Source<MatrixType>* Sprev )
	{
		if( rootType == CLIP_TYPE_NONE )
		{
			throw std::runtime_error( "Cannot set root for empty episode." );
		}
		else if( rootType == CLIP_TYPE_PREDICT )
		{
			predicts.front().SetRootSource( Sprev );
		}
		else if( rootType == CLIP_TYPE_UPDATE )
		{
			updates.front().SetRootSource( Sprev );
		}
	}

	percepto::Source<MatrixType>* GetTailSource()
	{
		if( tailType == CLIP_TYPE_NONE )
		{
			return &initCov;
		}
		else if( tailType == CLIP_TYPE_PREDICT )
		{
			return predicts.back().GetTailSource();
		}
		else if( tailType == CLIP_TYPE_UPDATE )
		{
			return updates.back().GetTailSource();
		}
		else
		{
			throw std::runtime_error( "Invalid tail type." );
		}
	}

	percepto::Source<double>* GetLL()
	{
		return &avgInnoLL;
	}

	// NOTE: We do not invalidate Sprev because we can't foreprop it
	void Invalidate()
	{
		// NOTE Shouldn't have to invalidate initCov, but to be safe...
		initCov.Invalidate();
		BOOST_FOREACH( KalmanFilterPredictModule& pred, predicts )
		{
			pred.Invalidate();
		}
		BOOST_FOREACH( KalmanFilterUpdateModule& upd, updates )
		{
			upd.Invalidate();
		}
	}

	void Foreprop()
	{
		initCov.Foreprop();
		BOOST_FOREACH( KalmanFilterPredictModule& pred, predicts )
		{
			pred.Foreprop();
		}
		BOOST_FOREACH( KalmanFilterUpdateModule& upd, updates )
		{
			upd.Foreprop();
		}
	}

private:
	// Forbid copying
	KalmanFilterEpisode( const KalmanFilterEpisode& other );

	// Forbid assigning
	KalmanFilterEpisode& operator=( const KalmanFilterEpisode& other );

};

std::ostream& operator<<( std::ostream& os, const KalmanFilterEpisode& episode )
{
	os << "Kalman filter episode:" << std::endl;
	unsigned int predInd = 0;
	unsigned int updInd = 0;
	for( unsigned int i = 0; i < episode.order.size(); i++ )
	{
		if( episode.order[i] == KalmanFilterEpisode::ClipType::CLIP_TYPE_PREDICT )
		{
			os << episode.predicts[ predInd ] << std::endl;
			predInd++;
		}
		else if( episode.order[i] == KalmanFilterEpisode::ClipType::CLIP_TYPE_UPDATE )
		{
			os << episode.updates[ updInd ] << std::endl;
			updInd++;
		}
	}
	return os;
}

struct InnovationLikelihoodProblem
{

	std::deque<KalmanFilterEpisode> episodes;
	percepto::StochasticMeanCost<double> loss;
	percepto::ParameterL2Cost regularizer;
	percepto::AdditiveWrapper<double> objective;

	InnovationLikelihoodProblem( percepto::Parameters* params,
	                             double l2Weight,
	                             unsigned int batchSize )
	{
		loss.SetBatchSize( batchSize );
		regularizer.SetParameters( params );
		regularizer.SetWeight( l2Weight );
		objective.SetSourceA( &loss );
		objective.SetSourceB( &regularizer );
	}

	template <class ...Args>
	void EmplaceEpisode( Args&& ...args )
	{
		episodes.emplace_back( args... );
		loss.AddSource( episodes.back().GetLL() );
	}

	size_t NumEpisodes() const { return episodes.size(); }

	KalmanFilterEpisode* GetCurrentEpisode()
	{
		if( episodes.empty() ) { return nullptr; }
		return &episodes.back();
	}

	const KalmanFilterEpisode* GetCurrentEpisode() const
	{
		if( episodes.empty() ) { return nullptr; }
		return &episodes.back();
	}

	void Invalidate()
	{
		regularizer.Invalidate();
		BOOST_FOREACH( KalmanFilterEpisode& ep, episodes )
		{
			ep.Invalidate();
		}
	}

	void Foreprop()
	{
		regularizer.Foreprop();
		BOOST_FOREACH( KalmanFilterEpisode& ep, episodes )
		{
			ep.Foreprop();
		}
		if( episodes.size() > 0 )
		{
			std::cout << "objective: " << objective.GetOutput() << std::endl;
		}
	}

	void Backprop()
	{
		objective.Backprop( MatrixType() );
	}

	double GetOutput() const
	{
		return objective.GetOutput();
	}

private:

	// Forbid copying
	InnovationLikelihoodProblem( const InnovationLikelihoodProblem& other );
	
	// Forbid assigning
	InnovationLikelihoodProblem& 
	operator=( const InnovationLikelihoodProblem& other );
};

std::ostream& operator<<( std::ostream& os, const InnovationLikelihoodProblem& problem )
{
	os << "Episodes: " << std::endl;
	for( unsigned int i = 0; i < problem.episodes.size(); i++ )
	{
		os << problem.episodes[i] << std::endl;
	}

	os << "regularizer penalty: " << problem.regularizer.GetOutput() << std::endl;
	os << "current objective: " << problem.objective.GetOutput() << std::endl;
	return os;
}

}