#pragma once

#include "fieldtrack/FieldtrackCommon.h"
#include "argus_utils/geometry/PoseSE3.h"
#include <string>
#include <unordered_set>
#include <unordered_map>

/*! \brief Contains various models and state representations for camera arrays. */
namespace argus
{

/*! \brief Represents a set of cameras. */
typedef std::unordered_set <std::string> CameraSet;

/*! \brief Describes the state of a camera array. Active and inactive cameras
 * are given as sets for easy querying and iterating. */
struct CameraArrayState
{
	CameraSet activeCameras;
	CameraSet inactiveCameras;
	
	bool IsActive( const std::string& name ) const;
	bool IsInactive( const std::string& name ) const;
	bool Deactivate( const std::string& name );
	bool Activate( const std::string& name );
};

/*! \brief An action that can be executed for a camera array. */
struct CameraArrayAction
{
	enum ActionType 
	{
		DO_NOTHING = 0,
		ACTIVATE_CAMERA,
		DEACTIVATE_CAMERA,
		SWITCH_CAMERAS
	};
	
	ActionType type;
	std::string toActivate;
	std::string toDeactivate;
	
};

std::ostream& operator<<( std::ostream& os, const CameraArrayAction& action );

// TODO Rename to TargetAction?
/*! \brief An acceleration command for a target. */
struct AccelerationAction
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	PoseSE3::TangentVector acceleration;
	
	/*! \brief Defaults to zero accelerations. */
	AccelerationAction();
};

/*! \brief Describes the state of the array, the carrier robot, and the relative
 * poses and body velocities of all tracked targets. */
struct RobotTargetState
{
	typedef std::unordered_map<std::string, TargetState> TargetMap;
	
	CameraArrayState array;
	TargetState robot;
	TargetMap targets;
};

/*! \brief Combined array action and acceleration actions for the robot and targets. */
struct RobotTargetAction
{
	typedef std::unordered_map<std::string, AccelerationAction> ActionMap;
	
	CameraArrayAction arrayAction;
	AccelerationAction robotAction;
	ActionMap targetActions;
};

}
