#include "fieldtrack/FieldtrackCommon.h"
#include "argus_utils/GeometryUtils.h"
#include "argus_utils/MatrixUtils.h"

using namespace argus;

namespace fieldtrack
{

TargetState::TargetState()
: velocity( VelocityType::Zero() ), poseCovariance( CovarianceType::Zero() ),
velocityCovariance( CovarianceType::Zero() )
{}
	
argus_msgs::CompactOdometry TargetToCompactOdom( const TargetState& state )
{
	argus_msgs::CompactOdometry odom;
	odom.pose.pose = PoseToMsg( state.pose );
	SerializeSymmetricMatrix( state.poseCovariance, odom.pose.covariance );
	odom.twist.twist = TangentToMsg( state.velocity );
	SerializeSymmetricMatrix( state.velocityCovariance, odom.twist.covariance );
	return odom;
}

TargetState CompactOdomToTarget( const argus_msgs::CompactOdometry& odom )
{
	TargetState state;
	state.pose = MsgToPose( odom.pose.pose );
	ParseSymmetricMatrix( odom.pose.covariance, state.poseCovariance );
	state.velocity = MsgToTangent( odom.twist.twist );
	ParseSymmetricMatrix( odom.twist.covariance, state.velocityCovariance );
	return state;
}

nav_msgs::Odometry TargetToOdom( const TargetState& state )
{
	nav_msgs::Odometry odom;
	odom.pose.pose = PoseToMsg( state.pose );
	SerializeMatrix( state.poseCovariance, odom.pose.covariance );
	odom.twist.twist = TangentToMsg( state.velocity );
	SerializeMatrix( state.velocityCovariance, odom.twist.covariance );
	return odom;	
}

TargetState OdomToTarget( const nav_msgs::Odometry& odom )
{
	TargetState state;
	state.pose = MsgToPose( odom.pose.pose );
	ParseMatrix( odom.pose.covariance, state.poseCovariance );
	state.velocity = MsgToTangent( odom.twist.twist );
	ParseMatrix( odom.twist.covariance, state.velocityCovariance );
	return state;
}
	
}
