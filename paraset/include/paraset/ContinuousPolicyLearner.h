#pragma once

#include "paraset/ContinuousPolicyManager.h"
#include "paraset/RewardStamped.h"
#include "paraset/ContinuousParamAction.h"
#include "paraset/PolicyLogGradientModules.h"

#include "paraset/ParasetInterfaces.h"

#include "argus_utils/synchronization/SynchronizationTypes.h"

#include <percepto/optim/OptimizerTypes.h>
#include <percepto/optim/ParameterL2Cost.hpp>
#include <percepto/optim/StochasticMeanCost.hpp>
#include <percepto/compo/AdditiveWrapper.hpp>

#include <deque>

namespace argus
{

// TODO Templatize on the cost term module
struct PolicyGradientOptimization
{
	std::deque<ContinuousLogGradientModule> modules;
	percepto::StochasticMeanCost<double> rewards;
	percepto::ParameterL2Cost regularizer;
	percepto::AdditiveWrapper<double> objective;

	PolicyGradientOptimization();

	void Initialize( percepto::Parameters::Ptr params,
	                 double l2Weight,
	                 unsigned int batchSize );

	template <class ... Args>
	void EmplaceModule( Args&&... args )
	{
		modules.emplace_back( args... );
		rewards.AddSource( modules.back().GetOutputSource() );
	}

	ContinuousLogGradientModule& GetLatestModule();
	void RemoveOldest();
	size_t NumModules() const;

	void Invalidate();
	void Foreprop();
	void ForepropAll();
	void Backprop();
	void BackpropNatural();

	double GetOutput() const;

	// Computes the mean log-likelihood of all the modules
	double ComputeLogProb();
};

struct PolicyDivergenceChecker
{
	PolicyGradientOptimization& optimization;
	double maxDivergence;
	double startingLogLikelihood;

	PolicyDivergenceChecker( PolicyGradientOptimization& opt );

	void SetDivergenceLimit( double m );
	void ResetDivergence();
	bool ExceededLimits();
};

class ContinuousPolicyLearner
{
public:

	ContinuousPolicyLearner();

	void Initialize( ros::NodeHandle& nh, ros::NodeHandle& ph );

private:

	mutable Mutex _mutex;

	ContinuousPolicyManager _manager;

	ros::Subscriber _actionSub;
	ros::Publisher _paramPub;

	PolicyCritic::Ptr _critic;

	ros::Timer _updateTimer;
	ros::Time _lastOptimizationTime;

	double _logdetWeight;

	double _actionBoundWeight;
	VectorType _scaledActionLowerLimit;
	VectorType _scaledActionUpperLimit;

	PolicyGradientOptimization _optimization;
	PolicyDivergenceChecker _optimizationChecker;

	typedef std::map<ros::Time, paraset::ContinuousParamAction> ActionBuffer;
	ActionBuffer _actionBuffer;

	bool _clearAfterOptimize;
	unsigned int _minModulesToOptimize;
	unsigned int _maxModulesToKeep;

	std::shared_ptr<percepto::AdamStepper> _stepper;
	std::shared_ptr<percepto::SimpleConvergence> _convergence;
	std::shared_ptr<percepto::AdamOptimizer> _optimizer;

	// std::shared_ptr<percepto::SimpleNaturalOptimizer> _optimizer;

	void ActionCallback( const paraset::ContinuousParamAction::ConstPtr& msg );
	void TimerCallback( const ros::TimerEvent& event );
};

}