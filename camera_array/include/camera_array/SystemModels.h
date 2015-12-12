#pragma once

#include "fieldtrack/FieldtrackCommon.h"
#include "camera_array/SystemStates.h"
#include "camera_array/ModelInterfaces.h"

namespace camera_array
{

typedef TransitionFunction <CameraArrayState, CameraArrayAction> ArrayTransitionFunction;
typedef TransitionFunction <TargetState, AccelerationAction> TargetTransitionFunction;
typedef TransitionFunction <RobotTargetState, CameraArrayAction> RobotArrayTransitionFunction;

typedef ActionGenerator <CameraArrayState, CameraArrayAction> ArrayActionGenerator;
typedef ActionGenerator <TargetState, AccelerationAction> TargetActionGenerator;

typedef RewardFunction <RobotTargetState, CameraArrayAction> RobotArrayReward;

typedef DecisionPolicy <RobotTargetState, CameraArrayAction> RobotArrayPolicy;

/*! \brief Transition function that transitions cameras instantly and
 * respects a ceiling on the max number of active cameras. */
class InstantCameraTransitionFunction
: public ArrayTransitionFunction
{
public:
	
	InstantCameraTransitionFunction( unsigned int maxCams );
	
	virtual CameraArrayState Transition( const CameraArrayState& state, const CameraArrayAction& action );
	
private:
	
	unsigned int maxActiveCams;
};

/*! \brief Simple action generator that allows camera activations when possible,
 * and switching all the time. */
class SwitchingCameraActionGenerator
: public ArrayActionGenerator
{
public:
	
	SwitchingCameraActionGenerator( unsigned int maxCams );
	
	virtual ActionList GetActions( const CameraArrayState& state );
	
private:
	
	unsigned int maxActiveCams;
	
};

/*! \brief Forward integrates a target motion. */
class FixedStepTargetTransitionFunction
: public TargetTransitionFunction
{
public:
	
	FixedStepTargetTransitionFunction( double timestep );
	
	virtual TargetState Transition( const TargetState& state,
	                                const AccelerationAction& action );
	
private:
	
	double dt;
};

class FixedTargetActionGenerator
: public TargetActionGenerator
{
public:
	
	FixedTargetActionGenerator( const AccelerationAction& out );
	
	virtual ActionList GetActions( const TargetState& state );
	
private:
	
	AccelerationAction output;
};

/*! \brief Combines an ArrayTransitionFunction with a TargetTransitionFunction 
 * to form a full RobotTargetTransitionFunction */
class MixedTransitionFunction
: public RobotArrayTransitionFunction
{
public:
	
	MixedTransitionFunction( const TargetTransitionFunction::Ptr& tfunc,
	                         const ArrayTransitionFunction::Ptr& afunc );
	
	virtual RobotTargetState Transition( const RobotTargetState& state,
	                                     const CameraArrayAction& action );
	
private:
	
	TargetTransitionFunction::Ptr targetTransition;
	ArrayTransitionFunction::Ptr arrayTransition;
};

};
