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
	std::deque<percepto::SquaredLoss<double>> penalties;
	std::deque<percepto::AdditiveWrapper<double>> modSums;

	percepto::StochasticMeanCost<double> loss;
	percepto::ParameterL2Cost regularizer;
	percepto::AdditiveWrapper<double> objective;

	double penaltyScale;

	ApproximateValueProblem();

	void Initialize( percepto::Parameters::Ptr params,
	                 double l2Weight,
	                 unsigned int sampleSize,
	                 double penaltyWeight );

	template <class ... Args>
	void EmplaceModule( Args&&... args )
	{
		modules.emplace_back( args... );

		penalties.emplace_back();
		penalties.back().SetSource( &modules.back().estValue->GetOutputSource() );
		penalties.back().SetTarget( 0.0 );
		penalties.back().SetScale( penaltyScale );

		modSums.emplace_back();
		modSums.back().SetSourceA( &modules.back().GetOutputSource() );
		modSums.back().SetSourceB( &penalties.back() );

		loss.AddSource( &modSums.back() );
	}

	void RemoveOldest();
	size_t NumModules() const;

	void Invalidate();
	void Foreprop();
	void Backprop();
	void BackpropNatural();

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
	// std::shared_ptr<percepto::SimpleNaturalOptimizer> _optimizer;
	unsigned int _optimCounter;

	bool _resetStepperAfter;
	unsigned int _minModulesToOptimize;
	unsigned int _maxModulesToKeep;
	bool _clearAfterOptimize;

	ApproximateValue _valueFunction;
	RewardInterpolater _rewardFunction;

	ros::Publisher _paramPub;
	
	ros::Timer _updateTimer;
	bool _timerInitialized;
	double _discountFactor;
	ros::Duration _timestep;

	ros::Duration _sampleOffset;
	ros::Duration _sampleTimestep;

	void UpdateCallback( const ros::TimerEvent& event );
	void SampleRange( const ros::Time& start, const ros::Time& end );
	void AddSample( const ros::Time& time );
	void RunOptimization();
};

}