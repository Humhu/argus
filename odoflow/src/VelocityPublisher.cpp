#include "odoflow/VelocityPublisher.h"

namespace argus
{
VelocityPublisher::VelocityPublisher( ros::NodeHandle& nh, ros::NodeHandle& ph )
{
	_velPub = ph.advertise<geometry_msgs::TwistStamped>( "velocity_raw", 10 );

	_minTimeDelta.InitializeAndRead( ph, 0.0, "min_time_delta",
	                                 "Min time between frames to estimate velocity" );
	_minTimeDelta.AddCheck<GreaterThanOrEqual>( 0.0 );
}

void VelocityPublisher::ReportPose( const ros::Time& time, const std::string& frameId,
                                    const PoseSE3& pose, double transScale )
{
	PoseSE3 disp = _lastPose.Inverse() * pose;
	double dt = (time - _lastTime).toSec();
	if( dt < _minTimeDelta ) { return; }

	_lastTime = time;
	_lastPose = pose;
	_lastVelocity = PoseSE3::Log( disp ) / dt;
	// NOTE Don't scale rotations by image scale!
	_lastVelocity.head<3>() *= transScale;

	geometry_msgs::TwistStamped tmsg;
	tmsg.header.stamp = time;
	tmsg.header.frame_id = frameId;
	tmsg.twist = TangentToMsg( _lastVelocity );
	_velPub.publish( tmsg );
}

const PoseSE3::TangentVector& VelocityPublisher::GetLastVelocity() const
{
	return _lastVelocity;
}

void VelocityPublisher::Reset( const ros::Time& time )
{
	_lastTime = time;
	_lastPose = PoseSE3();
	_lastVelocity.setZero();
}
}