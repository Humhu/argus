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
	percepto::MeanCost<double> loss;
	percepto::ParameterL2Cost regularizer;
	percepto::AdditiveWrapper<double> objective;

	PolicyGradientOptimization();

	void Initialize( percepto::Parameters::Ptr params,
	                 double l2Weight );

	template <class ... Args>
	void EmplaceModule( Args&&... args )
	{
		modules.emplace_back( args... );
		loss.AddSource( modules.back().GetOutputSource() );
	}

	void ClearModules();
	size_t NumModules() const;

	void Invalidate();
	void Foreprop();
	void Backprop();

	double GetOutput() const;
};

class ContinuousPolicyLearner
{
public:

	typedef ContinuousPolicyManager::NetworkType NetworkType;

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
	std::shared_ptr<PolicyGradientOptimization> _optimization;

	double _actionDelay;
	typedef std::map<ros::Time, paraset::ContinuousParamAction> ActionBuffer;
	ActionBuffer _actionBuffer;

	unsigned int _minModulesToOptimize;
	std::deque<ContinuousLogGradientModule> _gradModules;
	std::shared_ptr<percepto::AdamStepper> _stepper;
	std::shared_ptr<percepto::SimpleConvergence> _convergence;
	std::shared_ptr<percepto::AdamOptimizer> _optimizer;

	void ActionCallback( const paraset::ContinuousParamAction::ConstPtr& msg );
	void TimerCallback( const ros::TimerEvent& event );
	void InitializeOptimization();
};

}