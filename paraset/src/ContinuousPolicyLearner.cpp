#include "paraset/ContinuousPolicyLearner.h"
#include "argus_msgs/FloatVectorStamped.h"

#include "argus_utils/utils/ParamUtils.h"
#include "argus_utils/utils/MapUtils.hpp"

#include "paraset/DifferenceCritic.h"
#include "paraset/TDErrorCritic.h"

using namespace argus_msgs;

namespace argus
{

PolicyGradientOptimization::PolicyGradientOptimization() 
{}

void PolicyGradientOptimization::Initialize( percepto::Parameters::Ptr params,
                                             double l2Weight,
                                             unsigned int batchSize )
{
	// parameters = params;
	regularizer.SetParameters( params );
	regularizer.SetWeight( l2Weight );
	objective.SetSourceA( &rewards );
	objective.SetSourceB( &regularizer );
	rewards.SetBatchSize( batchSize );
}

size_t PolicyGradientOptimization::NumModules() const
{
	return modules.size();
}

void PolicyGradientOptimization::Invalidate()
{
	regularizer.Invalidate();
	BOOST_FOREACH( ContinuousLogGradientModule& mod, modules )
	{
		mod.Invalidate();
	}
}

void PolicyGradientOptimization::Foreprop()
{
	regularizer.Foreprop();
	rewards.Resample();
	const std::vector<unsigned int>& inds = rewards.GetActiveInds();
	BOOST_FOREACH( unsigned int ind, inds )
	// for( unsigned int ind = 0; ind < modules.size(); ++ind )
	{
		modules[ind].Foreprop();
	}
}

void PolicyGradientOptimization::ForepropAll()
{
	regularizer.Foreprop();
	BOOST_FOREACH( ContinuousLogGradientModule& mod, modules )
	{
		mod.Foreprop();
	}
}

void PolicyGradientOptimization::Backprop()
{
	objective.Backprop( MatrixType::Identity(1,1) );
}

void PolicyGradientOptimization::BackpropNatural()
{
	MatrixType back = MatrixType::Identity(1,1) / modules.size();
	BOOST_FOREACH( ContinuousLogGradientModule& mod, modules )
	{
		mod.GetLogProbSource()->Backprop( back );
	}
}

double PolicyGradientOptimization::GetOutput() const
{
	double out = objective.GetOutput();
	ROS_INFO_STREAM( "Objective: " << out );
	return out;
}

double PolicyGradientOptimization::ComputeLogProb()
{
	ForepropAll();
	double acc = 0;
	BOOST_FOREACH( ContinuousLogGradientModule& mod, modules )
	{
		acc += mod.GetLogProbSource()->GetOutput();
	}
	return acc / modules.size();
}

void PolicyGradientOptimization::RemoveOldest()
{
	rewards.RemoveOldestSource();
	modules.pop_front();
}

ContinuousLogGradientModule& PolicyGradientOptimization::GetLatestModule()
{
	return modules.back();
}

PolicyDivergenceChecker::PolicyDivergenceChecker( PolicyGradientOptimization& opt )
: optimization( opt ) {}

void PolicyDivergenceChecker::SetDivergenceLimit( double m )
{
	maxDivergence = m;
}

void PolicyDivergenceChecker::ResetDivergence()
{
	startingLogLikelihood = optimization.ComputeLogProb();
}

bool PolicyDivergenceChecker::ExceededLimits()
{
	double div = std::abs( optimization.ComputeLogProb() - startingLogLikelihood );
	ROS_INFO_STREAM( "Divergence: " << div );
	if( !std::isfinite( div ) )
	{
		throw std::runtime_error( "Non-finite divergence." );
	}
	return div > maxDivergence;
}

ContinuousPolicyLearner::ContinuousPolicyLearner() 
: _optimizationChecker( _optimization ) {}

void ContinuousPolicyLearner::Initialize( ros::NodeHandle& nh, ros::NodeHandle& ph )
{
	WriteLock lock( _mutex );
	
	ros::NodeHandle ih( ph.resolveName( "policy" ) );
	_manager.Initialize( nh, ih );

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
	percepto::AdamParameters stepperParams;
	// percepto::NaturalStepperParameters stepperParams;
	GetParam( lh, "stepper/step_size", stepperParams.alpha, 1E-3 );
	GetParam( lh, "stepper/max_step", stepperParams.maxStepElement, 1.0 );
	// GetParam( lh, "stepper/beta1", stepperParams.beta1, 0.9 );
	// GetParam( lh, "stepper/beta2", stepperParams.beta2, 0.99 );
	// GetParam( lh, "stepper/epsilon", stepperParams.epsilon, 1E-7 );

	// double windowRatio;
	// GetParam( lh, "stepper/window_ratio", windowRatio, 1.0 );
	// stepperParams.windowLen = std::ceil( windowRatio * _manager.GetParameters()->ParamDim() );
	// ROS_INFO_STREAM( "Initializing natural gradient window with " << stepperParams.windowLen <<
	                 // " sample length." );

	GetParam( lh, "stepper/enable_decay", stepperParams.enableDecay, false );
	_stepper = std::make_shared<percepto::AdamStepper>( stepperParams );
	_convergence = std::make_shared<percepto::SimpleConvergence>( criteria );
	_optimizer = std::make_shared<percepto::AdamOptimizer>( *_stepper, 
	                                                          *_convergence,
	                                                          *_manager.GetParameters(),
	                                                          percepto::OPT_MAXIMIZATION );
	// _optimizer = std::make_shared<percepto::SimpleNaturalOptimizer>( *_convergence,
	//                                                                  *_manager.GetParameters(),
	//                                                                  stepperParams,
	//                                                                  percepto::OPT_MAXIMIZATION );

	_scaledActionLowerLimit = ( _manager.GetPolicyInterface().GetLowerLimits().array() / _manager.GetScales() ).matrix();
	_scaledActionUpperLimit = ( _manager.GetPolicyInterface().GetUpperLimits().array() / _manager.GetScales() ).matrix();
	ROS_INFO_STREAM( "Scaled lower bounds: " << _scaledActionLowerLimit.transpose() );
	ROS_INFO_STREAM( "Scaled upper bounds: " << _scaledActionUpperLimit.transpose() );

	double l2Weight;
	unsigned int batchSize;
	GetParamRequired( lh, "l2_weight", l2Weight );
	GetParamRequired( lh, "batch_size", batchSize );
	_optimization.Initialize( _manager.GetParameters(), l2Weight, batchSize );

	double maxDivergence;
	GetParamRequired( lh, "max_divergence", maxDivergence );
	_optimizationChecker.SetDivergenceLimit( maxDivergence );
	percepto::AdamOptimizer::UserCheck divCheck = boost::bind( &PolicyDivergenceChecker::ExceededLimits, 
	                                                           &_optimizationChecker );
	_optimizer->AddUserCheck( divCheck );

	ros::NodeHandle ch( ph.resolveName( "critic" ) );
	std::string criticType;
	GetParamRequired( ch, "type", criticType );
	if( criticType == "difference" )
	{
		_critic = std::make_shared<DifferenceCritic>();
		_critic->Initialize( nh, ch );
	}
	else if( criticType == "td_error" )
	{
		_critic = std::make_shared<TDErrorCritic>();
		_critic->Initialize( nh, ch );
	}
	else
	{
		throw std::invalid_argument( "ContinuousPolicyLearner: Unsupported critic type: " + criticType );
	}

	_paramPub = ph.advertise<FloatVectorStamped>( "param_updates", 1 );
	_actionSub = nh.subscribe( "actions", 
	                           0, 
	                           &ContinuousPolicyLearner::ActionCallback, 
	                           this );

	_lastOptimizationTime = ros::Time::now();
	double updateRate;
	GetParamRequired( lh, "update_rate", updateRate );
	_updateTimer = nh.createTimer( ros::Duration( 1.0/updateRate ),
	                               &ContinuousPolicyLearner::TimerCallback,
	                               this );
}

void ContinuousPolicyLearner::ActionCallback( const paraset::ContinuousParamAction::ConstPtr& msg )
{
	if( msg->header.stamp < _lastOptimizationTime ) { return; }

	// TODO Check for identical timestamps
	ContinuousParamAction action( *msg );
	_actionBuffer[ msg->header.stamp ] = *msg;
}

void ContinuousPolicyLearner::TimerCallback( const ros::TimerEvent& event )
{
	// NOTE Always publish parameters to ensure synchronization
	StampedFeatures update( event.current_real, "", _manager.GetParameters()->GetParamsVec() );
	_paramPub.publish( update.ToMsg() );

	// Wait until we have enough actions buffered before we begin optimization
	if( _actionBuffer.size() < _minModulesToOptimize ) 
	{
		ROS_INFO_STREAM( "Action buffer size: " << _actionBuffer.size() << 
		                 " less than min: " << _minModulesToOptimize );
		return;
	}

	// First process buffer
	ros::Time now = event.current_real;
	while( _actionBuffer.size() > 0 )
	{
		ContinuousParamAction action( _actionBuffer.begin()->second );
		action.output = ( (action.output - _manager.GetOffsets()).array() / _manager.GetScales() ).matrix();

		double advantage;
		try
		{
			advantage = _critic->Evaluate( action );
		}
		catch( std::out_of_range )
		{
			ROS_WARN_STREAM( "Could not evaluate action at time: " << action.time );
			break;
		}

		remove_lowest( _actionBuffer );
		if( !action.input.allFinite() )
		{
			ROS_WARN_STREAM( "Received non-finite input: " << action.input.transpose() );
			continue;
		}
		if( !action.output.allFinite() )
		{
			ROS_WARN_STREAM( "Received non-finite action: " << action.output.transpose() );
			continue;
		}

		if( !std::isfinite( advantage ) )
		{
			ROS_WARN_STREAM( "Received non-finite advantage: " << advantage );
			continue;
		}
		_optimization.EmplaceModule( _manager.GetPolicyModule(),
		                             action.input, 
		                             action.output,
		                             advantage );

		_critic->Publish( action );
		ROS_INFO_STREAM( "Action: " << action.output.transpose() << 
		                 " Input: " << action.input.transpose() << 
		                 " Advantage: " << advantage );
	}

	if( _optimization.NumModules() == 0 ) 
	{
		_actionBuffer.clear();
		return;
	}

	// Perform optimization
	ROS_INFO_STREAM( "Optimizing with " << _optimization.NumModules() << " modules." );
	// _stepper->Reset();
	_optimizationChecker.ResetDivergence();
	percepto::OptimizationResults results = _optimizer->Optimize( _optimization );

	ROS_INFO_STREAM( "Objective: " << results.finalObjective );
	ROS_INFO_STREAM( "Policy: " << *_manager.GetPolicyModule() );
	// ROS_INFO_STREAM( "Gradients: " << _manager.GetParameters()->GetDerivs() );

	// Now that we've changed the policy, these old actions are no longer valid
	_lastOptimizationTime = ros::Time::now();
	_actionBuffer.clear();

	// Trim old data
	unsigned int targetModules = _maxModulesToKeep;
	if( _clearAfterOptimize )
	{
		targetModules = 0;
	}
	while( _optimization.NumModules() > targetModules )
	{
		_optimization.RemoveOldest();
	}
}

}