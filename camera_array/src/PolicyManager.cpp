#include "camera_array/PolicyManager.h"
#include "argus_utils/utils/ParamUtils.h"
#include "fieldtrack/FieldtrackCommon.h"

#include "camera_array/StochasticGreedyPolicy.hpp"
#include "camera_array/ExpectationPolicy.h"

namespace argus
{

PolicyManager::PolicyManager( const ros::NodeHandle& nh, const ros::NodeHandle& ph,
                              const CameraArrayManager::Ptr& cam )
: nodeHandle( nh ), privHandle( ph ), manager( cam )
{
	// TODO Temporarily hard-coded types here
	unsigned int maxCams;
	GetParam<unsigned int>( privHandle, "max_active_cameras", maxCams, 1 );
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
	
	// Reward function sampling
	rewardFunction = std::make_shared<FiducialRewardFunction>( fiducialModel, systemTransitionFunction );
	
	unsigned int numSamples;
	GetParam<unsigned int>( privHandle, "num_samples", numSamples, 10 );
	
	policy = std::make_shared<ExpectationPolicy>( rewardFunction, numSamples );
	
	targetSub = nodeHandle.subscribe( "target_states", 
	                                  1,
	                                  &PolicyManager::TargetCallback, 
	                                  this );
	
	odomSub = nodeHandle.subscribe( "carrier_state",
	                                1,
	                                &PolicyManager::OdometryCallback,
	                                this );
	
	double updateRate;
	ph.param<double>( "policy_update_rate", updateRate, 1.0 );
	updateTimer = std::make_shared<ros::Timer>
		( nodeHandle.createTimer( ros::Duration( 1.0/updateRate ),
	                              &PolicyManager::TimerCallback,
		                          this ) );
}

void PolicyManager::OdometryCallback( const nav_msgs::Odometry::ConstPtr& msg )
{
	lastOdometry = msg;
}

void PolicyManager::TargetCallback( const argus_msgs::OdometryArray::ConstPtr& msg )
{
	lastTargets = msg;
}

void PolicyManager::TimerCallback( const ros::TimerEvent& event )
{
	// TODO Fast-forward states to current time
	
	if( !lastOdometry  ) { return; }
	
	RobotTargetState state;
	state.array = manager->GetState();
	state.robot = TargetState( *lastOdometry );
	
	if( lastTargets )
	{
		for( unsigned int i = 0; i < lastTargets->odometry.size(); i++ )
		{
			const std::string& name = lastTargets->odometry[i].child_frame_id;
			state.targets[ name ] = TargetState( lastTargets->odometry[i] );
		}
	}
	
	std::vector<CameraArrayAction> arrayActions = arrayActionGenerator->GetActions( state.array );
	
	CameraArrayAction action = policy->ChooseAction( state, arrayActions );
	CameraSet activeReference = state.array.activeCameras;
	switch( action.type )
	{
		case CameraArrayAction::ACTIVATE_CAMERA:
			activeReference.insert( action.toActivate );
			ROS_INFO_STREAM( "Activating camera " << action.toActivate );
			manager->SetActiveCameras( activeReference );
			break;
		case CameraArrayAction::DEACTIVATE_CAMERA:
			activeReference.erase( activeReference.find( action.toDeactivate ) );
			ROS_INFO_STREAM( "Deactivating camera " << action.toDeactivate );
			break;
		case CameraArrayAction::SWITCH_CAMERAS:
			activeReference.insert( action.toActivate );
			activeReference.erase( activeReference.find( action.toDeactivate ) );
			ROS_INFO_STREAM( "Switching camera " << action.toDeactivate << 
				" to " << action.toActivate );
			break;
		case CameraArrayAction::DO_NOTHING:
			ROS_INFO_STREAM( "Doing nothing." );
			break;
	}
	manager->SetActiveCameras( activeReference );
}
	
}
