#include "fieldtrack/FieldtrackCommon.h"
#include "argus_utils/geometry/GeometryUtils.h"
#include "argus_utils/utils/MatrixUtils.h"

#define POSE_DIM (PoseSE3::TangentDimension)

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

TargetState::TargetState() {}

TargetState::TargetState( const fieldtrack::TargetState& state )
{
	referenceFrame = state.header.frame_id;
	bodyFrame = state.body_frame_id;
	timestamp = state.header.stamp;

	pose = MsgToPose( state.pose );
	derivatives = GetVectorView( state.derivatives );

	unsigned int N = POSE_DIM + derivatives.size();
	covariance = MatrixType( N, N );
	ParseMatrix( state.covariance, covariance );
}

TargetState::TargetState( const nav_msgs::Odometry& odom )
{
	referenceFrame = odom.header.frame_id;
	bodyFrame = odom.child_frame_id;
	timestamp = odom.header.stamp;

	pose = MsgToPose( odom.pose.pose );
	derivatives = MsgToTangent( odom.twist.twist );

	covariance = MatrixType( 2 * POSE_DIM, 2 * POSE_DIM );
	MatrixType temp( POSE_DIM, POSE_DIM );
	ParseMatrix( odom.pose.covariance, temp );
	covariance.topLeftCorner( POSE_DIM, POSE_DIM ) = temp;
	ParseMatrix( odom.twist.covariance, temp );
	covariance.bottomRightCorner( POSE_DIM, POSE_DIM ) = temp;
}

nav_msgs::Odometry TargetState::ToOdometryMsg() const
{
	nav_msgs::Odometry odom;
	odom.header.frame_id = referenceFrame;
	odom.header.stamp = timestamp;
	odom.child_frame_id = bodyFrame;

	odom.pose.pose = PoseToMsg( pose );
	if( derivatives.size() < POSE_DIM )
	{
		throw std::invalid_argument( "Cannot convert TargetState with no derivatives to odom." );
	}

	// Need to explicitly call the PoseSE3::TangentVector version of TangentToMsg
	PoseSE3::TangentVector velocity = derivatives.head<POSE_DIM>();
	odom.twist.twist = TangentToMsg( velocity );

	SerializeMatrix( covariance.block<POSE_DIM, POSE_DIM>( 0, 0 ),
	                 odom.pose.covariance );
	SerializeMatrix( covariance.block<POSE_DIM, POSE_DIM>( POSE_DIM, POSE_DIM ),
	                 odom.twist.covariance );
	return odom;
}

fieldtrack::TargetState TargetState::ToStateMsg() const
{
	fieldtrack::TargetState msg;
	msg.header.frame_id = referenceFrame;
	msg.header.stamp = timestamp;
	msg.body_frame_id = bodyFrame;

	msg.pose = PoseToMsg( pose );
	SerializeMatrix( derivatives, msg.derivatives );
	SerializeMatrix( covariance, msg.covariance );
	return msg;
}

struct ObsMsgTimeVisitor
	: public boost::static_visitor<ros::Time>
{
	ObsMsgTimeVisitor() {}

	template<typename M>
	ros::Time operator()( const M& msg ) const
	{
		return msg.header.stamp;
	}
};

ros::Time get_timestamp( const ObservationMessage& msg )
{
	return boost::apply_visitor( ObsMsgTimeVisitor(), msg );
}

struct ObsTimeVisitor
	: public boost::static_visitor<ros::Time>
{
	ObsTimeVisitor() {}

	ros::Time operator()( const ObservationBase& obs ) const
	{
		return obs.timestamp;
	}
};

ros::Time get_timestamp( const Observation& obs )
{
	return boost::apply_visitor( ObsTimeVisitor(), obs );
}

struct ObsFrameVisitor
	: public boost::static_visitor<std::string>
{
	ObsFrameVisitor() {}

	std::string operator()( const ObservationBase& obs ) const
	{
		return obs.referenceFrame;
	}
};

std::string get_frame( const Observation& obs )
{
	return boost::apply_visitor( ObsFrameVisitor(), obs );
}
}
