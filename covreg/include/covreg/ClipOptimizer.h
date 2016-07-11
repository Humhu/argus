#pragma once

#include <percepto/optim/OptimizerTypes.h>
#include <percepto/optim/ParameterL2Cost.hpp>

#include <argus_utils/utils/LinalgTypes.h>
#include <argus_utils/filters/FilterInfo.h>

#include <memory>
#include <unordered_map>

#include <boost/foreach.hpp>

#include "covreg/InnovationProblem.h"
#include "covreg/CovarianceEstimator.h"
#include "argus_utils/synchronization/SynchronizationTypes.h"

namespace argus
{

// TODO Get rid of this? There aren't that many parameters
struct InnovationClipParameters
{
	unsigned int maxEpisodeLength;
	double l2Weight;
	unsigned int batchSize;

	InnovationClipParameters()
	: maxEpisodeLength( 10 ),
	l2Weight( 1E-9 ), batchSize( 30 ) {}
};

class InnovationClipOptimizer
{
public:

	InnovationClipOptimizer( CovarianceEstimator& qReg,
	                         const InnovationClipParameters& params =
	                         InnovationClipParameters() );

	void AddObservationReg( CovarianceEstimator& reg, const std::string& name );

	void AddPredict( const PredictInfo& info, const VectorType& input );

	bool AddUpdate( const UpdateInfo& info, const VectorType& input,
	                const std::string& name, double weight, const ros::Time& stamp );

	// Terminates the current episode and starts a new one
	void BreakCurrentEpisode();
	void RemoveEarliestEpisode();
	ros::Time GetEarliestTime();

	void InitializeOptimization( const percepto::SimpleConvergenceCriteria& criteria,
	                             const percepto::AdamParameters& params );
	// Returns whether it converged or bailed early
	bool Optimize();

	size_t NumEpisodes() const;
	size_t CurrentEpisodeLength() const;

	double CalculateCost();
	void Print( std::ostream& os );

private:

	// Forbid copying and moving
	InnovationClipOptimizer( const InnovationClipOptimizer& other );
	InnovationClipOptimizer& operator=( const InnovationClipOptimizer& other );

	mutable Mutex _mutex;
	
	KalmanFilterEpisode* _currentEpisode;
	std::vector< std::pair<PredictInfo,VectorType> > _predBuffer;

	// The regressors optimized
	CovarianceEstimator& _transReg;
	std::unordered_map <std::string, CovarianceEstimator&> _obsRegs;

	// The optimization problem
	InnovationLikelihoodProblem _problem;
	
	unsigned int _maxEpisodeLength;

	// All parameters from optimized estimators
	percepto::ParameterWrapper _paramWrapper;

	// Optimization
	std::shared_ptr<percepto::AdamStepper> _stepper;
	std::shared_ptr<percepto::SimpleConvergence> _convergence;
	std::shared_ptr<percepto::AdamOptimizer> _optimizer;

};

std::ostream& operator<<( std::ostream& os, InnovationClipOptimizer& opt );

}