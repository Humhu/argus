#include "paraset/ContinuousPolicyLearner.h"
#include "argus_msgs/FloatVectorStamped.h"

#include "argus_utils/utils/ParamUtils.h"
#include "argus_utils/utils/MapUtils.hpp"

#include "paraset/DifferenceCritic.h"

using namespace argus_msgs;

namespace argus
{

PolicyGradientOptimization::PolicyGradientOptimization() 
{}

void PolicyGradientOptimization::Initialize( percepto::Parameters::Ptr params,
                                             double l2Weight )
{
	// parameters = params;
	regularizer.SetParameters( params );
	regularizer.SetWeight( l2Weight );
	objective.SetSourceA( &rewards );
	objective.SetSourceB( &regularizer );
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
	VectorType scales( modules.size() );
	for( unsigned int i = 0; i < modules.size(); ++i )
	{
		scales(i) = modules[i].logExpectedAdvantage.GetScale();
		modules[i].logExpectedAdvantage.SetScale( 1.0 );
	}
	double l2 = regularizer.GetWeight();
	regularizer.SetWeight( 0 );

	Backprop();

	for( unsigned int i = 0; i < modules.size(); ++i )
	{
		modules[i].logExpectedAdvantage.SetScale( scales(i) );
	}
	regularizer.SetWeight( l2 );
}

double PolicyGradientOptimization::GetOutput() const
{
	return objective.GetOutput();
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

ContinuousPolicyLearner::ContinuousPolicyLearner() {}

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
	// percepto::DirectStepperParameters stepperParams;
	percepto::NaturalStepperParameters stepperParams;
	GetParam( lh, "stepper/step_size", stepperParams.alpha, 1E-3 );
	GetParam( lh, "stepper/max_step", stepperParams.maxStepElement, 1.0 );
	// GetParam( lh, "stepper/beta1", stepperParams.beta1, 0.9 );
	// GetParam( lh, "stepper/beta2", stepperParams.beta2, 0.99 );
	GetParam<unsigned int>( lh, "stepper/window_len", stepperParams.windowLen, 100 );
	GetParam( lh, "stepper/epsilon", stepperParams.epsilon, 1E-7 );
	GetParam( lh, "stepper/enable_decay", stepperParams.enableDecay, false );
	// _stepper = std::make_shared<percepto::DirectStepper>( stepperParams );
	_convergence = std::make_shared<percepto::SimpleConvergence>( criteria );
	// _optimizer = std::make_shared<percepto::DirectOptimizer>( *_stepper, 
	//                                                           *_convergence,
	//                                                           *_manager.GetParameters(),
	//                                                           percepto::OPT_MAXIMIZATION );
	_optimizer = std::make_shared<percepto::SimpleNaturalOptimizer>( *_convergence,
	                                                                 *_manager.GetParameters(),
	                                                                 stepperParams,
	                                                                 percepto::OPT_MAXIMIZATION );

	_scaledActionLowerLimit = ( _manager.GetPolicyInterface().GetLowerLimits().array() / _manager.GetScales() ).matrix();
	_scaledActionUpperLimit = ( _manager.GetPolicyInterface().GetUpperLimits().array() / _manager.GetScales() ).matrix();
	ROS_INFO_STREAM( "Scaled lower bounds: " << _scaledActionLowerLimit.transpose() );
	ROS_INFO_STREAM( "Scaled upper bounds: " << _scaledActionUpperLimit.transpose() );

	GetParamRequired( lh, "l2_weight", _l2Weight );
	InitializeOptimization();

	GetParamRequired( lh, "action_time_delay", _actionDelay );

	ros::NodeHandle ch( ph.resolveName( "critic" ) );
	std::string criticType;
	GetParamRequired( ch, "type", criticType );
	if( criticType == "difference" )
	{
		_critic = std::make_shared<DifferenceCritic>();
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

	double updateRate;
	GetParamRequired( lh, "update_rate", updateRate );
	_updateTimer = nh.createTimer( ros::Duration( 1.0/updateRate ),
	                               &ContinuousPolicyLearner::TimerCallback,
	                               this );
}

void ContinuousPolicyLearner::InitializeOptimization()
{
	_optimization = std::make_shared<PolicyGradientOptimization>();
	_optimization->Initialize( _manager.GetParameters(), _l2Weight );
}

void ContinuousPolicyLearner::ActionCallback( const paraset::ContinuousParamAction::ConstPtr& msg )
{
	// Record the action
	ContinuousParamAction action( *msg );
	// TODO Check for identical timestamps
	_actionBuffer[ msg->header.stamp ] = *msg;
}

void ContinuousPolicyLearner::TimerCallback( const ros::TimerEvent& event )
{
	// First process buffer
	ros::Time now = event.current_real;
	while( _actionBuffer.size() > 0 && ( now - get_lowest_key( _actionBuffer ) ).toSec() > _actionDelay )
	{
		ContinuousParamAction action( _actionBuffer.begin()->second );
		remove_lowest( _actionBuffer );
		action.output = ( (action.output - _manager.GetOffsets()).array() / _manager.GetScales() ).matrix();

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

		double advantage;
		try
		{
			advantage = _critic->Evaluate( action );
		}
		catch( std::out_of_range )
		{
			ROS_WARN_STREAM( "Could not evaluate action at time: " << action.time );
			continue;
		}

		if( !std::isfinite( advantage ) )
		{
			ROS_WARN_STREAM( "Received non-finite advantage: " << advantage );
			continue;
		}

		// _optimization->EmplaceModule( _manager.GetPolicyModule(),
		//                               action.input, 
		//                               action.output,
		//                               advantage,
		//                               _scaledActionLowerLimit,
		//                               _scaledActionUpperLimit,
		//                               _actionBoundWeight );
		_optimization->EmplaceModule( _manager.GetPolicyModule(),
		                              action.input, 
		                              action.output,
		                              advantage );


		_critic->Publish( action );
		ROS_INFO_STREAM( "Action: " << action.output.transpose() << " advantage: " << advantage );
	}

	// Then perform optimization
	if( _optimization->NumModules() < _minModulesToOptimize ) 
	{
		ROS_INFO_STREAM( "Num modules: " << _optimization->NumModules() << 
		                 " less than min: " << _minModulesToOptimize );
	}
	else
	{
		percepto::OptimizationResults results = _optimizer->Optimize( *_optimization );

		ROS_INFO_STREAM( "Objective: " << results.finalObjective );
		ROS_INFO_STREAM( "Policy: " << *_manager.GetPolicyModule() );
		// ROS_INFO_STREAM( "Gradients: " << _manager.GetParameters()->GetDerivs() );

		if( _clearAfterOptimize )
		{
			InitializeOptimization();
		}
		else
		{
			while( _optimization->NumModules() > _maxModulesToKeep )
			{
				_optimization->RemoveOldest();
			}
		}
	}

	StampedFeatures update( event.current_real, "", _manager.GetParameters()->GetParamsVec() );
	_paramPub.publish( update.ToMsg() );
}

}