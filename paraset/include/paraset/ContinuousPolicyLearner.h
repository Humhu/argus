#pragma once

#include "paraset/ContinuousPolicyManager.h"
#include "paraset/RewardStamped.h"
#include "paraset/ContinuousParamAction.h"
#include "paraset/PolicyLogGradientModules.h"

#include "paraset/ParasetInterfaces.h"

#include "argus_utils/synchronization/SynchronizationTypes.h"

#include <percepto/optim/OptimizerTypes.h>
#include <percepto/optim/ParameterL2Cost.hpp>
#include <percepto/optim/MeanCost.hpp>
#include <percepto/compo/AdditiveWrapper.hpp>

#include <deque>

namespace argus
{

// TODO Templatize on the cost term module
struct PolicyGradientOptimization
{
	std::deque<ContinuousLogGradientModule> modules;
	percepto::MeanCost<double> rewards;
	percepto::ParameterL2Cost regularizer;
	percepto::AdditiveWrapper<double> objective;

	PolicyGradientOptimization();

	void Initialize( percepto::Parameters::Ptr params,
	                 double l2Weight );

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
	void Backprop();
	void BackpropNatural();

	double GetOutput() const;

	percepto::Parameters::Ptr parameters;;
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

	double _l2Weight;
	double _logdetWeight;

	double _actionBoundWeight;
	VectorType _scaledActionLowerLimit;
	VectorType _scaledActionUpperLimit;

	std::shared_ptr<PolicyGradientOptimization> _optimization;

	double _actionDelay;
	typedef std::map<ros::Time, paraset::ContinuousParamAction> ActionBuffer;
	ActionBuffer _actionBuffer;

	bool _clearAfterOptimize;
	unsigned int _minModulesToOptimize;
	unsigned int _maxModulesToKeep;

	std::deque<ContinuousLogGradientModule> _gradModules;
	std::shared_ptr<percepto::DirectStepper> _stepper;
	std::shared_ptr<percepto::SimpleConvergence> _convergence;
	// std::shared_ptr<percepto::DirectOptimizer> _optimizer;
	std::shared_ptr<percepto::SimpleNaturalOptimizer> _optimizer;

	void ActionCallback( const paraset::ContinuousParamAction::ConstPtr& msg );
	void TimerCallback( const ros::TimerEvent& event );
	void InitializeOptimization();
};

}