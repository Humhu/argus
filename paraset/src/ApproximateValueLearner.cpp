#include "paraset/ApproximateValueLearner.h"
#include <boost/foreach.hpp>

namespace argus
{

ApproximateValueProblem::ApproximateValueProblem() {}

void ApproximateValueProblem::Initialize( percepto::Parameters::Ptr params,
                                          double l2Weight,
                                          unsigned int sampleSize,
                                          double penaltyWeight )
{
	penaltyScale = penaltyWeight;
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
	penalties.pop_front();
	modSums.pop_front();
}

size_t ApproximateValueProblem::NumModules() const
{
	return modules.size();
}

void ApproximateValueProblem::Invalidate()
{
	regularizer.Invalidate();
	for( unsigned int i = 0; i < NumModules(); ++i )
	{
		modules[i].Invalidate();
		penalties[i].Invalidate();
	}
}

void ApproximateValueProblem::Foreprop()
{
	regularizer.Foreprop();
	
	loss.Resample();
	const std::vector<unsigned int>& inds = loss.GetActiveInds();
	BOOST_FOREACH( unsigned int ind, inds )
	// for( unsigned int ind = 0; ind < modules.size(); ind++ )
	{
		modules[ind].Foreprop();
		penalties[ind].Foreprop();
	}
}

void ApproximateValueProblem::Backprop()
{
	objective.Backprop( MatrixType::Identity(1,1) );
}

void ApproximateValueProblem::BackpropNatural()
{
	MatrixType back = MatrixType::Identity(1,1) / modules.size();
	BOOST_FOREACH( BellmanResidualModule& module, modules )
	{
		module.Foreprop();
		// NOTE estValue is used in the regularizer term, so we can't backprop it directly
		module.nextValue->GetOutputSource().Backprop( back );
	}
}

double ApproximateValueProblem::GetOutput() const
{
	double out = objective.GetOutput();
	// ROS_INFO_STREAM( "Objective: " << out );
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

	// percepto::NaturalStepperParameters stepperParams;
	percepto::AdamParameters stepperParams;
	GetParam( lh, "stepper/step_size", stepperParams.alpha, 1E-3 );
	GetParam( lh, "stepper/max_step", stepperParams.maxStepElement, 1.0 );
	GetParam( lh, "stepper/beta1", stepperParams.beta1, 0.9 );
	GetParam( lh, "stepper/beta2", stepperParams.beta2, 0.99 );
	GetParam( lh, "stepper/epsilon", stepperParams.epsilon, 1E-7 );
	
	// double windowRatio;
	// GetParam( lh, "stepper/window_ratio", windowRatio, 1.0 );
	// stepperParams.windowLen = std::ceil( windowRatio * _valueFunction.GetParameters()->ParamDim() );

	GetParam( lh, "stepper/enable_decay", stepperParams.enableDecay, false );
	GetParam( lh, "stepper/reset_after_optimization", _resetStepperAfter, false );
	_stepper = std::make_shared<percepto::AdamStepper>( stepperParams );
	_optimizer = std::make_shared<percepto::AdamOptimizer>( *_stepper, 
	                                                          *_convergence,
	                                                          *_valueFunction.GetParameters(),
	                                                          percepto::OPT_MINIMIZATION );
	// _optimizer = std::make_shared<percepto::SimpleNaturalOptimizer>( *_convergence,
	//                                                                  *_valueFunction.GetParameters(),
	//                                                                  stepperParams,
	//                                                                  percepto::OPT_MINIMIZATION );
	_optimCounter = 0;

	_paramPub = ph.advertise<argus_msgs::FloatVectorStamped>( "param_updates", 1 );

	double l2Weight, valuePenaltyWeight;
	unsigned int batchSize;
	GetParamRequired( lh, "l2_weight", l2Weight );
	GetParamRequired( lh, "batch_size", batchSize );
	GetParamRequired( lh, "value_penalty_weight", valuePenaltyWeight );
	_problem.Initialize( _valueFunction.GetParameters(), 
	                     l2Weight, 
	                     batchSize, 
	                     valuePenaltyWeight );

	// TODO GetParam specialization for ros::Duration and ros::Rate
	_timerInitialized = false;
	double updateRate, discountRate, sampleOffset, dt, sampleRate;
	GetParamRequired( ph, "update_rate", updateRate );
	GetParamRequired( ph, "sample_offset", sampleOffset );
	GetParamRequired( ph, "sample_rate", sampleRate );
	GetParamRequired( ph, "timestep", dt );
	_sampleOffset = ros::Duration( sampleOffset );
	_timestep = ros::Duration( dt );
	_sampleTimestep = ros::Duration( 1.0/sampleRate );

	if( GetParam( ph, "discount_rate", discountRate ) )
	{
		_discountFactor = std::exp( dt * std::log( discountRate ) );
		ROS_INFO_STREAM( "Computed discount factor of: " << _discountFactor << 
		                 " from desired rate: " << discountRate );
	}
	else
	{
		GetParamRequired( ph, "discount_factor", _discountFactor );
	}

	_updateTimer = nh.createTimer( ros::Duration( 1.0/updateRate ),
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

	SampleRange( event.last_expected, event.current_expected );
	RunOptimization();

	StampedFeatures update( event.current_real, "", _valueFunction.GetParameters()->GetParamsVec() );
	_paramPub.publish( update.ToMsg() );
}


void ApproximateValueLearner::RunOptimization()
{
	if( _problem.NumModules() < _minModulesToOptimize )
	{
		ROS_INFO_STREAM( "Num modules: " << _problem.NumModules() <<
		                 " less than min: " << _minModulesToOptimize );
		return;
	}

	if( _resetStepperAfter )
	{
		// _stepper->Reset();
	}

	percepto::OptimizationResults results = _optimizer->Optimize( _problem );
	ROS_INFO_STREAM( "Objective: " << results.finalObjective );
	_optimCounter++;
	if( _optimCounter % 10 == 0 )
	{
		ROS_INFO_STREAM( "Approximator: " << _valueFunction.CreateApproximatorModule()->Print() );
	}

	unsigned int targetNum = _maxModulesToKeep;
	if( _clearAfterOptimize )
	{
		targetNum = 0;
	}
	while( _problem.NumModules() > targetNum )
	{
		_problem.RemoveOldest();
	}
}

void ApproximateValueLearner::SampleRange( const ros::Time& start, const ros::Time& end )
{
	ros::Time curr = start;
	while( curr < end )
	{
		AddSample( curr );
		curr += _sampleTimestep;
	}
}

void ApproximateValueLearner::AddSample( const ros::Time& time )
{
	ros::Time currTime = time - _sampleOffset;
	ros::Time prevTime = currTime - _timestep;

	double reward;
	VectorType currInput, prevInput;
	try
	{
		reward = _rewardFunction.IntegratedReward( prevTime, currTime );
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
}

}