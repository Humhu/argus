#include "camera_array/PolicyManager.h"
#include "argus_utils/ParamUtils.h"
#include "fieldtrack/FieldtrackCommon.h"

using namespace argus_utils;

namespace camera_array
{

PolicyManager::PolicyManager( const ros::NodeHandle& nh, const ros::NodeHandle& ph,
                              const CameraArrayManager::Ptr& cam )
: nodeHandle( nh ), privHandle( ph ), manager( cam )
{
	// TODO Temporarily hard-coded types here
	unsigned int maxCams;
	GetParamDefault<unsigned int>( privHandle, "max_active_cameras", maxCams, 1 );
	arrayTransitionFunction = std::make_shared<InstantCameraTransitionFunction>( maxCams );
	
	double lookaheadDt;
	privHandle.param<double>( "lookahead_dt", lookaheadDt, 0.2 );
	targetTransitionFunction = std::make_shared<FixedStepTargetTransitionFunction>( lookaheadDt );
	
	systemTransitionFunction = 
		std::make_shared<MixedTransitionFunction>( targetTransitionFunction,
		                                           arrayTransitionFunction );
	
	arrayActionGenerator = std::make_shared<SwitchingCameraActionGenerator>( maxCams );
		
	// TODO Set up lookup namespace
	fiducialModel = std::make_shared<FiducialDetectionModel>( lookupInterface );
	rewardFunction = std::make_shared<FiducialRewardFunction>( fiducialModel, systemTransitionFunction );
		
	double rewardScale;
	ph.param<double>( "reward_scale", rewardScale, 1.0 );
	policy = std::make_shared<Policy>( rewardFunction, rewardScale );
	
	targetSub = nodeHandle.subscribe( "target_states", 
	                                  1,
	                                  &PolicyManager::TargetCallback, 
	                                  this );
	
	odomSub = nodeHandle.subscribe( "carrier_state",
	                                1,
	                                &PolicyManager::OdometryCallback,
	                                this );
	
	double updateRate;
	ph.param<double>( "update_rate", updateRate, 1.0 );
	updateTimer = std::make_shared<ros::Timer>
		( nodeHandle.createTimer( ros::Duration( 1.0/updateRate ),
	                              &PolicyManager::TimerCallback,
		                          this ) );
}

void PolicyManager::OdometryCallback( const nav_msgs::Odometry::ConstPtr& msg )
{
	lastOdometry = msg;
}

void PolicyManager::TargetCallback( const argus_msgs::CompactOdometryArray::ConstPtr& msg )
{
	lastTargets = msg;
}

void PolicyManager::TimerCallback( const ros::TimerEvent& event )
{
	// TODO Fast-forward states to current time
	
	if( !lastOdometry  ) { return; }
	
	RobotTargetState state;
	state.array = manager->GetState();
	state.robot = fieldtrack::OdomToTarget( *lastOdometry );
	
	if( lastTargets )
	{
		for( unsigned int i = 0; i < lastTargets->odometry.size(); i++ )
		{
			const std::string& name = lastTargets->odometry[i].child_frame_id;
			state.targets[ name ] = fieldtrack::CompactOdomToTarget( lastTargets->odometry[i] );
		}
	}
	
	std::vector<CameraArrayAction> arrayActions = arrayActionGenerator->GetActions( state.array );
	
	CameraArrayAction action = policy->ChooseAction( state, arrayActions );
	switch( action.type )
	{
		case CameraArrayAction::ACTIVATE_CAMERA:
			ROS_INFO_STREAM( "Activating camera " << action.toActivate );
			manager->RequestSetStreaming( action.toActivate, true );
			break;
		case CameraArrayAction::DEACTIVATE_CAMERA:
			ROS_INFO_STREAM( "Deactivating camera " << action.toActivate );
			manager->RequestSetStreaming( action.toDeactivate, false );
			break;
		case CameraArrayAction::SWITCH_CAMERAS:
			ROS_INFO_STREAM( "Switching camera " << action.toDeactivate << 
				" to " << action.toActivate );
			manager->RequestSetStreaming( action.toDeactivate, false );
			manager->RequestSetStreaming( action.toActivate, true );
			break;
		case CameraArrayAction::DO_NOTHING:
			ROS_INFO_STREAM( "Doing nothing." );
			break;
	}
}
	
}