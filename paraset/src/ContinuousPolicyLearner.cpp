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
	regularizer.SetParameters( params );
	regularizer.SetWeight( l2Weight );
	objective.SetSourceA( &loss );
	objective.SetSourceB( &regularizer );
}

void PolicyGradientOptimization::ClearModules()
{
	modules.clear();
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

double PolicyGradientOptimization::GetOutput() const
{
	return objective.GetOutput();
}

ContinuousPolicyLearner::ContinuousPolicyLearner() {}

void ContinuousPolicyLearner::Initialize( ros::NodeHandle& nh, ros::NodeHandle& ph )
{
	WriteLock lock( _mutex );
	
	ros::NodeHandle ih( ph.resolveName( "policy" ) );
	_manager.Initialize( nh, ih );


	ros::NodeHandle lh( ph.resolveName( "optimization" ) );
	percepto::SimpleConvergenceCriteria criteria;
	GetParamRequired( lh, "min_num_modules", _minModulesToOptimize );
	GetParam( lh, "convergence/max_time", criteria.maxRuntime, std::numeric_limits<double>::infinity() );
	GetParam( lh, "convergence/max_iters", criteria.maxIterations, std::numeric_limits<unsigned int>::max() );
	GetParam( lh, "convergence/min_avg_delta", criteria.minAverageDelta, -std::numeric_limits<double>::infinity() );
	GetParam( lh, "convergence/min_avg_grad", criteria.minAverageGradient, -std::numeric_limits<double>::infinity() );
	percepto::AdamParameters stepperParams;
	GetParam( lh, "stepper/step_size", stepperParams.alpha, 1E-3 );
	GetParam( lh, "stepper/max_step", stepperParams.maxStepElement, 1.0 );
	GetParam( lh, "stepper/beta1", stepperParams.beta1, 0.9 );
	GetParam( lh, "stepper/beta2", stepperParams.beta2, 0.99 );
	GetParam( lh, "stepper/epsilon", stepperParams.epsilon, 1E-7 );
	GetParam( lh, "stepper/enable_decay", stepperParams.enableDecay, false );
	_stepper = std::make_shared<percepto::AdamStepper>( stepperParams );
	_convergence = std::make_shared<percepto::SimpleConvergence>( criteria );
	_optimizer = std::make_shared<percepto::AdamOptimizer>( *_stepper, 
	                                                        *_convergence,
	                                                        *_manager.GetParameters(),
	                                                        percepto::AdamOptimizer::OPT_MAXIMIZATION );

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

		if( !action.output.allFinite() )
		{
			ROS_WARN_STREAM( "Received non-finite action: " << action.output.transpose() );
			continue;
		}
		else if( !action.input.allFinite() )
		{
			ROS_WARN_STREAM( "Received non-finite input: " << action.input.transpose() );
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

		_critic->Publish( action );
		_optimization->EmplaceModule( _manager.GetPolicyModule(),
                                      action.input, 
                                      action.output,
                                      advantage );
		ROS_INFO_STREAM( "Action: " << action.output.transpose() << " advantage: " << advantage );
	}

	// Then perform optimization
	if( _optimization->NumModules() < _minModulesToOptimize ) 
	{
		ROS_INFO_STREAM( "Num modules: " << _optimization->NumModules() << 
		                 " less than min: " << _minModulesToOptimize );
		return; 
	}
	percepto::OptimizationResults results = _optimizer->Optimize( *_optimization );

	ROS_INFO_STREAM( "Objective: " << results.finalObjective );
	ROS_INFO_STREAM( "Policy: " << _manager.GetPolicyModule() );

	StampedFeatures update( event.current_real, "", _manager.GetParameters()->GetParamsVec() );
	_paramPub.publish( update.ToMsg() );

	// Reinitialize to clear the modules
	InitializeOptimization();
}

}