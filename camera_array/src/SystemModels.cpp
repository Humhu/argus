#include "camera_array/SystemModels.h"
#include <boost/foreach.hpp>

using namespace argus;

namespace camera_array
{

InstantCameraTransitionFunction::InstantCameraTransitionFunction( unsigned int maxCams )
: maxActiveCams( maxCams ) {}

CameraArrayState InstantCameraTransitionFunction::Transition( const CameraArrayState& state,
                                                              const CameraArrayAction& action )
{
	CameraArrayState next( state );
	
	switch( action.type )
	{
		case CameraArrayAction::DO_NOTHING:
		
			// Do as it says!
			break;
		
		case CameraArrayAction::ACTIVATE_CAMERA:
			
			// If the target is not active and we have fewer than max, it activates
			if( state.IsInactive( action.toActivate ) &&
				state.activeCameras.size() < maxActiveCams )
			{
				next.Activate( action.toActivate );
			}
			
			// Else nothing happens
			break;

		case CameraArrayAction::DEACTIVATE_CAMERA:
			
			// If the target is active, it deactivates
			if( state.IsActive( action.toDeactivate ) )
			{
				next.Deactivate( action.toDeactivate );
			}
			
			// Else nothing happens
			break;
			
		case CameraArrayAction::SWITCH_CAMERAS:
		
			// If the target cameras are in their valid states, switch them
			if( state.IsActive( action.toDeactivate ) &&
				state.IsInactive( action.toActivate ) )
			{
				next.Activate( action.toActivate );
				next.Deactivate( action.toDeactivate );
			}
			
			// Else nothing happens
			break;
	}
	return next;
}

SwitchingCameraActionGenerator::SwitchingCameraActionGenerator( unsigned int maxCams )
: maxActiveCams( maxCams ) {}

SwitchingCameraActionGenerator::ActionList
SwitchingCameraActionGenerator::GetActions( const CameraArrayState& state )
{
	ActionList actions;
	
	CameraArrayAction action;
	
	// First activate all cameras if allowed
	if( state.activeCameras.size() < maxActiveCams )
	{
		action.type = CameraArrayAction::ACTIVATE_CAMERA;
		BOOST_FOREACH( const std::string& name, state.inactiveCameras )
		{
			action.toActivate = name;
			actions.push_back( action );
		}
	}
	// Otherwise allow null action
	else
	{
		action.type = CameraArrayAction::DO_NOTHING;
		actions.push_back( action );	
	}
	
	// Now switch for all combinations
	action.type = CameraArrayAction::SWITCH_CAMERAS;
	BOOST_FOREACH( const std::string& active, state.activeCameras )
	{
		action.toDeactivate = active;
		BOOST_FOREACH( const std::string& inactive, state.inactiveCameras )
		{
			action.toActivate = inactive;
			actions.push_back( action );
		}
	}
	
	return actions;
}

FixedStepTargetTransitionFunction::FixedStepTargetTransitionFunction( double timestep )
: dt( timestep ) 
{}

TargetState 
FixedStepTargetTransitionFunction::Transition( const TargetState& state,
                                               const AccelerationAction& action )
{
	TargetState next( state );
	PoseSE3 displacement = PoseSE3::Exp( dt * state.velocity + 
	                                     0.5 * dt * dt * action.acceleration );
	next.pose = state.pose * displacement;
	next.velocity = state.velocity + dt * action.acceleration;
	return next;
}

FixedTargetActionGenerator::FixedTargetActionGenerator( const AccelerationAction& o )
: output( o ) {}

std::vector<AccelerationAction> FixedTargetActionGenerator::GetActions( const TargetState& state )
{
	std::vector<AccelerationAction> outputs;
	outputs.push_back( output );
	return outputs;
}

MixedTransitionFunction::MixedTransitionFunction( const TargetTransitionFunction::Ptr& tfunc,
                                                  const ArrayTransitionFunction::Ptr& afunc )
: targetTransition( tfunc ), arrayTransition( afunc ) {}

RobotTargetState MixedTransitionFunction::Transition( const RobotTargetState& state,
                                                      const CameraArrayAction& action )
{
	RobotTargetState next( state );
	next.array = arrayTransition->Transition( state.array, action );
	
	AccelerationAction defaultAcceleration;
	next.robot = targetTransition->Transition( state.robot, defaultAcceleration );
// 	BOOST_FOREACH( const RobotTargetAction::ActionMap::value_type& item, action.targetActions )
	BOOST_FOREACH( const RobotTargetState::TargetMap::value_type& item, state.targets )
	{
		const std::string& targetName = item.first;
// 		const AccelerationAction& targetAction = item.second;
		AccelerationAction targetAction;
		next.targets[ targetName ] = targetTransition->Transition( state.targets.at( targetName ), 
		                                                           targetAction );
	}
	return next;
}

}
