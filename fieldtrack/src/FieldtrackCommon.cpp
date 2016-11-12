#include "fieldtrack/FieldtrackCommon.h"
#include "argus_utils/geometry/GeometryUtils.h"
#include "argus_utils/utils/MatrixUtils.h"

namespace argus
{

TargetState::TargetState()
: pose(),
  poseCovariance( PoseSE3::CovarianceMatrix::Zero() ),
  velocity( PoseSE3::TangentVector::Zero() ), 
  velocityCovariance( PoseSE3::CovarianceMatrix::Zero() )
{}

TargetState::TargetState( const nav_msgs::Odometry& odom )
{
	referenceFrame = odom.header.frame_id;
	bodyFrame = odom.child_frame_id;
	timestamp = odom.header.stamp;
	pose = MsgToPose( odom.pose.pose );
	ParseMatrix( odom.pose.covariance, poseCovariance );
	velocity = MsgToTangent( odom.twist.twist );
	ParseMatrix( odom.twist.covariance, velocityCovariance );
}

nav_msgs::Odometry TargetState::ToMsg() const
{
	nav_msgs::Odometry odom;
	odom.header.frame_id = referenceFrame;
	odom.header.stamp = timestamp;
	odom.child_frame_id = bodyFrame;
	odom.pose.pose = PoseToMsg( pose );
	SerializeMatrix( poseCovariance, odom.pose.covariance );
	odom.twist.twist = TangentToMsg( velocity );
	SerializeMatrix( velocityCovariance, odom.twist.covariance );
	return odom;
}

FilterUpdate::FilterUpdate() {}

FilterUpdate::FilterUpdate( const argus_msgs::FilterUpdate& msg )
{
	sourceName = msg.header.frame_id;
	timestamp = msg.header.stamp;
	observationMatrix = MsgToMatrix( msg.observation_matrix );
	observationCov = MsgToMatrix( msg.observation_cov );
	observation = GetVectorView( msg.observation );
}

argus_msgs::FilterUpdate FilterUpdate::ToMsg() const
{
	argus_msgs::FilterUpdate msg;
	msg.header.frame_id = sourceName;
	msg.header.stamp = timestamp;
	msg.observation_matrix = MatrixToMsg( observationMatrix );
	msg.observation_cov = MatrixToMsg( observationCov );
	SerializeMatrix( observation, msg.observation );
	return msg;
}

}
