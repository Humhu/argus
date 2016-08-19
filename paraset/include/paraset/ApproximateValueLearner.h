#pragma once

#include "paraset/ApproximateValue.h"
#include "paraset/ValueResidualModules.h"
#include "paraset/RewardInterpolater.h"

#include "percepto/optim/OptimizerTypes.h"
#include "percepto/optim/ParameterL2Cost.hpp"
#include "percepto/optim/StochasticMeanCost.hpp"
#include "percepto/compo/AdditiveWrapper.hpp"

namespace argus
{

struct ApproximateValueProblem
{
	std::deque<BellmanResidualModule> modules;
	percepto::StochasticMeanCost<double> loss;
	percepto::ParameterL2Cost regularizer;
	percepto::AdditiveWrapper<double> objective;

	ApproximateValueProblem();

	void Initialize( percepto::Parameters::Ptr params,
	                 double l2Weight,
	                 unsigned int sampleSize );

	template <class ... Args>
	void EmplaceModule( Args&&... args )
	{
		modules.emplace_back( args... );
		loss.AddSource( &modules.back().GetOutputSource() );
	}

	void RemoveOldest();
	size_t NumModules() const;

	void Invalidate();
	void Foreprop();
	void Backprop();

	double GetOutput() const;

};

class ApproximateValueLearner
{
public:

	ApproximateValueLearner();

	void Initialize( ros::NodeHandle& nh, ros::NodeHandle& ph );

private:

	ApproximateValueProblem _problem;

	std::shared_ptr<percepto::AdamStepper> _stepper;
	std::shared_ptr<percepto::SimpleConvergence> _convergence;
	std::shared_ptr<percepto::AdamOptimizer> _optimizer;

	unsigned int _minModulesToOptimize;
	unsigned int _maxModulesToKeep;
	bool _clearAfterOptimize;
	double _discountFactor;

	ApproximateValue _valueFunction;
	RewardInterpolater _rewardFunction;

	bool _timerInitialized;
	ros::Publisher _paramPub;
	ros::Duration _updatePeriod;
	ros::Timer _updateTimer;

	void UpdateCallback( const ros::TimerEvent& event );

};

}