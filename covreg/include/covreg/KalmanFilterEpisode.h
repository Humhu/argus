#pragma once

#include <percepto/optim/StochasticSumCost.hpp>
#include <percepto/optim/MeanCost.hpp>

#include "covreg/KalmanFilterModules.h"
#include <ros/ros.h>

namespace argus
{
	
class KalmanFilterEpisode
{
public:
	// We use deques here so that growing the container does not invalidate
	// any self-pointers used the the predict/update modules
	
	percepto::TerminalSource<VectorType> initState;

	// The estimate covariance at the beginning of the episode
	percepto::TerminalSource<MatrixType> initCov;

	// The predict steps in this episode
	std::deque<KalmanFilterPredictModule> predicts;

	// The update steps in this episode
	std::deque<KalmanFilterUpdateModule> updates;

	// This forces the sumInnoLL to always be able to output, even with 0 updates
	percepto::TerminalSource<double> offsetInnoLL;

	// Scaled LLs for each source
	std::deque<percepto::ScaleWrapper<double>> llScales;

	percepto::MeanCost<double> sumInnoLL;

	ros::Time startTime;

	enum ClipType
	{
		CLIP_TYPE_NONE = 0,
		CLIP_TYPE_PREDICT,
		CLIP_TYPE_UPDATE
	};

	// Order of predicts/updates for printing
	std::vector<ClipType> order;

	ClipType tailType;

	KalmanFilterEpisode( const VectorType& xinit, const MatrixType& Sinit, const ros::Time& t );

	size_t NumUpdates() const;

	template <class ...Args>
	void EmplacePredict( Args&&... args )
	{
		predicts.emplace_back( args... );
		tailType = CLIP_TYPE_PREDICT;
		order.push_back( CLIP_TYPE_PREDICT );
		// predicts.back().Q.modName = "Q";
		// predicts.back().Qdt.modName = "Qdt";
		// predicts.back().Sminus.modName = "Sminus";
		// predicts.back().FSFT.modName = "FSFT";
	}

	template <class ...Args>
	void EmplaceUpdate( const std::string& name, 
	                    double scale,
	                    Args&&... args )
	{
		updates.emplace_back( args... );
		updates.back().sourceName = name;
		// updates.back().Vinvv.modName = "Vinvv";
		// updates.back().innov.modName = "innov";
		// updates.back().xcorr.modName = "xcorr";
		// updates.back().V.modName = "V";
		// updates.back().xplus.modName = "xplus";
		// updates.back().Vinv.modName = "Vinv";
		// updates.back().R.modName = "R";
		// updates.back().Splus.modName = "Splus";
		// updates.back().HTVinvH.modName = "HTVinvH";

		llScales.emplace_back();
		llScales.back().SetScale( scale );
		llScales.back().SetSource( &updates.back().innovationLL );
		//sumInnoLL.AddSource( &updates.back().innovationLL );
		sumInnoLL.AddSource( &llScales.back());

		tailType = CLIP_TYPE_UPDATE;
		order.push_back( CLIP_TYPE_UPDATE );
	}

	percepto::Source<VectorType>* GetTailState();
	percepto::Source<MatrixType>* GetTailCov();

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