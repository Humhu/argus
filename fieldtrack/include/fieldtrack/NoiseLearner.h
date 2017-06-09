 #pragma once

#include <ros/ros.h>

#include "fieldtrack/CovarianceModels.h"
#include "fieldtrack/LikelihoodChain.h"

#include "modprop/compo/BasicModules.h"

#include "argus_utils/synchronization/SynchronizationTypes.h"
#include <deque>
#include <map>

namespace argus
{
class NoiseLearner
{
public:

	NoiseLearner();

	// Read parameters from a ROS node handle
	void Initialize( ros::NodeHandle& ph );

	// Register transition and observation models to the current chain
	// NOTE This should be called before starting any new chains!
	void RegisterTransModel( const CovarianceModel::Ptr& model );
	void RegisterObsModel( const std::string& name,
	                       const CovarianceModel::Ptr& model );
	
	void BufferInfo( const FilterInfo& info );
	void ClearBuffer();

	// Caps the current chain and begins a new one
	// NOTE Clears any FilterInfo in the buffer after capping
	void StartNewChain();
	
	// Deletes all chains
	void ClearChains();

	// Learns parameters on capped chains
	void LearnSpin();

	unsigned int NumCappedChains() const;

private:

	unsigned int _minNumModules;
	unsigned int _maxBufferSize;

	unsigned int _numIterations;
	double _stepDecayRate;

	unsigned int _stepsPerBatch;
	double _stepSize;
	double _maxL1Norm;

	mutable Mutex _bufferMutex; // Controls access to buffer
	mutable Mutex _chainMutex; // Controls access to chain
	std::deque<FilterInfo> _infoBuffer;

	std::deque<LikelihoodChain::Ptr> _chains;
	MeanModule _meanChainLL;
	SinkModule _meanLL;

	CovarianceModel::Ptr _transCov;
	typedef std::map<std::string, CovarianceModel::Ptr> ObsModelRegistry;
	ObsModelRegistry _obsModels;

	// Computes the current step size
	double GetStepSize() const;

	// Returns the total parameter size over all models
	unsigned int GetParamDim() const;
	
	// Foreprop and backprops all capped chains
	// Assumes _chainMutex acquired externally
	void RunChains();
	void Invalidate();
	void Foreprop();

	// Retrieves derivatives from all registered models
	VectorType GetDerivatives() const;

	// Increments parameters by specified amount
	void StepParameters( const VectorType& s );

	// Assumes _chainMutex acquired externally
	// TODO Check external lock
	LikelihoodChain& GetCurrentChain();

	void ProcessBuffer( WriteLock& cLock );

	template <typename Lock>
	unsigned int NumCappedChains( Lock& lock ) const
	{
		CheckLockOwnership( lock, &_chainMutex );
		if( _chains.empty() ) { return 0; }
		return _chains.size() - 1;
	}
};
}