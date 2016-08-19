#include "paraset/ApproximateValueLearner.h"
#include <boost/foreach.hpp>

namespace argus
{

ApproximateValueProblem::ApproximateValueProblem() {}

void ApproximateValueProblem::Initialize( percepto::Parameters::Ptr params,
                                          double l2Weight,
                                          unsigned int sampleSize )
{
	regularizer.SetParameters( params );
	regularizer.SetWeight( l2Weight );
	objective.SetSourceA( &loss );
	objective.SetSourceB( &regularizer );
	loss.SetBatchSize( sampleSize );
}

void ApproximateValueProblem::RemoveOldest()
{
	loss.RemoveOldestSource();
	modules.pop_front();
}

size_t ApproximateValueProblem::NumModules() const
{
	return modules.size();
}

void ApproximateValueProblem::Invalidate()
{
	regularizer.Invalidate();
	BOOST_FOREACH( BellmanResidualModule& mod, modules )
	{
		mod.Invalidate();
	}
}

void ApproximateValueProblem::Foreprop()
{
	regularizer.Foreprop();
	
	loss.Resample();
	const std::vector<unsigned int>& inds = loss.GetActiveInds();
	BOOST_FOREACH( unsigned int ind, inds )
	{
		modules[ind].Foreprop();
	}
}

void ApproximateValueProblem::Backprop()
{
	objective.Backprop( MatrixType::Identity(1,1) );
}

double ApproximateValueProblem::GetOutput() const
{
	double out = objective.GetOutput();
	return out;
}

ApproximateValueLearner::ApproximateValueLearner() {}

void ApproximateValueLearner::Initialize( ros::NodeHandle& nh, ros::NodeHandle& ph )
{
	ros::NodeHandle vh( ph.resolveName( "value_function" ) );
	_valueFunction.Initialize( nh, vh );

	ros::NodeHandle rh( ph.resolveName( "reward_function" ) );
	_rewardFunction.Initialize( nh, rh );
	
	ros::NodeHandle lh( ph.resolveName( "optimization" ) );
	GetParamRequired( lh, "min_num_modules", _minModulesToOptimize );
	GetParam( lh, "clear_optimized_modules", _clearAfterOptimize, false );
	if( !_clearAfterOptimize )
	{
		GetParamRequired( lh, "max_num_modules", _maxModulesToKeep );
	}

	percepto::SimpleConvergenceCriteria criteria;
	GetParam( lh, "convergence/max_time", criteria.maxRuntime, std::numeric_limits<double>::infinity() );
	GetParam( lh, "convergence/max_iters", criteria.maxIterations, std::numeric_limits<unsigned int>::max() );
	GetParam( lh, "convergence/min_avg_delta", criteria.minAverageDelta, -std::numeric_limits<double>::infinity() );
	GetParam( lh, "convergence/min_avg_grad", criteria.minAverageGradient, -std::numeric_limits<double>::infinity() );
	_convergence = std::make_shared<percepto::SimpleConvergence>( criteria );

	percepto::AdamParameters stepperParams;
	GetParam( lh, "stepper/step_size", stepperParams.alpha, 1E-3 );
	GetParam( lh, "stepper/max_step", stepperParams.maxStepElement, 1.0 );
	GetParam( lh, "stepper/beta1", stepperParams.beta1, 0.9 );
	GetParam( lh, "stepper/beta2", stepperParams.beta2, 0.99 );
	GetParam( lh, "stepper/epsilon", stepperParams.epsilon, 1E-7 );
	GetParam( lh, "stepper/enable_decay", stepperParams.enableDecay, false );
	_stepper = std::make_shared<percepto::AdamStepper>( stepperParams );
	_optimizer = std::make_shared<percepto::AdamOptimizer>( *_stepper, 
	                                                          *_convergence,
	                                                          *_valueFunction.GetParameters(),
	                                                          percepto::OPT_MINIMIZATION );

	_paramPub = ph.advertise<argus_msgs::FloatVectorStamped>( "param_updates", 1 );

	double l2Weight;
	GetParamRequired( lh, "l2_weight", l2Weight );
	unsigned int batchSize;
	GetParamRequired( lh, "batch_size", batchSize );
	_problem.Initialize( _valueFunction.GetParameters(), l2Weight, batchSize );

	_timerInitialized = false;
	double updateRate, discountRate;
	GetParamRequired( ph, "update_rate", updateRate );
	GetParamRequired( ph, "discount_rate", discountRate );

	_updatePeriod = ros::Duration( 1.0/updateRate );
	_discountFactor = std::exp( 1.0/updateRate * std::log( discountRate ) );
		ROS_INFO_STREAM( "Computed discount factor of: " << _discountFactor << 
		                 " from desired rate: " << discountRate );
	_updateTimer = nh.createTimer( _updatePeriod,
	                               &ApproximateValueLearner::UpdateCallback,
	                               this );
}

void ApproximateValueLearner::UpdateCallback( const ros::TimerEvent& event )
{
	if( !_timerInitialized )
	{
		_timerInitialized = true;
		return;
	}

	ros::Time currTime = event.current_expected;
	ros::Time prevTime = event.last_expected;

	double reward;
	VectorType currInput, prevInput;
	try
	{
		reward = _rewardFunction.Evaluate( prevTime );
		currInput = _valueFunction.GetInput( currTime );
		prevInput = _valueFunction.GetInput( prevTime );
	} 
	catch( std::out_of_range )
	{
		ROS_WARN_STREAM( "Could not evaluate reward or actions at current time: " << currTime );
		return;
	}

	if( !std::isfinite( reward ) )
	{
		ROS_WARN_STREAM( "Received non-finite reward: " << reward );
		return;
	}

	_problem.EmplaceModule( _valueFunction.CreateApproximatorModule(),
	                        prevInput,
	                        _valueFunction.CreateApproximatorModule(),
	                        currInput,
	                        reward,
	                        _discountFactor );

	if( _problem.NumModules() < _minModulesToOptimize )
	{
		ROS_INFO_STREAM( "Num modules: " << _problem.NumModules() <<
		                 " less than min: " << _minModulesToOptimize );
		return;
	}

	percepto::OptimizationResults results = _optimizer->Optimize( _problem );
	ROS_INFO_STREAM( "Objective: " << results.finalObjective );
	ROS_INFO_STREAM( "Approximator: " << _valueFunction.CreateApproximatorModule()->Print() );

	unsigned int targetNum = _maxModulesToKeep;
	if( _clearAfterOptimize )
	{
		targetNum = 0;
	}
	while( _problem.NumModules() > targetNum )
	{
		_problem.RemoveOldest();
	}

	StampedFeatures update( event.current_real, "", _valueFunction.GetParameters()->GetParamsVec() );
	_paramPub.publish( update.ToMsg() );
}

}