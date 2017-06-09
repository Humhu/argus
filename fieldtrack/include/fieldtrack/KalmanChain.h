#pragma once

#include "modprop/kalman/kalman.hpp"
#include "argus_utils/filter/FilterInfo.h"

#include <memory>
#include <deque>

namespace argus
{
class KalmanChain
{
public:

	typedef std::shared_ptr<KalmanChain> Ptr;
	typedef std::shared_ptr<PredictModule> PredictModulePtr;
	typedef std::shared_ptr<UpdateModule> UpdateModulePtr;
	typedef std::pair<PredictModulePtr, UpdateModulePtr> ModulePtrPair;

	KalmanChain();
	virtual ~KalmanChain();

	void Initialize( const VectorType& x0, const MatrixType& P0 );

	virtual void Foreprop();
	virtual void Invalidate();

	size_t NumModules() const;
	size_t NumPredicts() const;
	size_t NumUpdates() const;

	// NOTE The proper thing to do is return a boost::variant but I am super lazy
	ModulePtrPair RemoveEarliest();
	void Clear();

	std::shared_ptr<PredictModule> AddLinearPredict( const MatrixType& A );
	std::shared_ptr<UpdateModule> AddLinearUpdate( const MatrixType& C, const VectorType& y );

private:

	enum ChainType
	{
		CHAIN_PREDICT,
		CHAIN_UPDATE
	};

	KalmanPrior _prior;

	std::deque<ChainType> _types;
	std::deque<PredictModulePtr> _predicts;
	std::deque<UpdateModulePtr> _updates;

	KalmanOut& GetLastModule();
	KalmanIn& GetFirstModule();
};
}