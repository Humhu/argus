#include "fieldtrack/FieldtrackCommon.h"
#include "argus_utils/geometry/GeometryUtils.h"
#include "argus_utils/utils/MatrixUtils.h"

namespace argus
{

CovarianceMode StringToCovMode( const std::string& str )
{
	if( str == "pass" ) { return COV_PASS; }
	if( str == "fixed" ) { return COV_FIXED; }
	if( str == "adaptive" ) { return COV_ADAPTIVE; }
	else
	{
		throw std::invalid_argument( "Unknown covariance mode: " + str );
	}
}

std::string CovModeToString( CovarianceMode mode )
{
	if( mode == COV_PASS ) { return "pass"; }
	if( mode == COV_FIXED ) { return "fixed"; }
	if( mode == COV_ADAPTIVE ) { return "adaptive"; }
	else
	{
		throw std::invalid_argument( "Unknown covariance mode: " + mode );
	}
}

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

}
