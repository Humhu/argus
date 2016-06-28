#include <ros/ros.h>

#include "nav_msgs/Odometry.h"
#include "geometry_msgs/TwistStamped.h"

#include "argus_utils/utils/ParamUtils.h"
#include "argus_utils/geometry/GeometryUtils.h"

using namespace argus;

class OdometryDifferentiator
{
public:

	OdometryDifferentiator( ros::NodeHandle& nh, ros::NodeHandle& ph )
	: initialized( false )
	{
		unsigned int buffLen;
		GetParam<unsigned int>( ph, "buffer_length", buffLen, 10 );
		double dt;
		GetParam<double>( ph, "min_dt", dt, 0.1 );
		_minDt = ros::Duration( dt );

		_twistPub = nh.advertise<geometry_msgs::TwistStamped>( "velocity", 
		                                                  buffLen );
		_odomSub = nh.subscribe( "odom", 
		                         buffLen, 
		                         &OdometryDifferentiator::OdomCallback, 
		                         this );
	}

private:

	ros::Publisher _twistPub;
	ros::Subscriber _odomSub;
	PoseSE3 _lastPose;
	ros::Time _lastPoseTime;
	bool initialized;
	ros::Duration _minDt;

	void OdomCallback( const nav_msgs::Odometry::ConstPtr& msg )
	{
		PoseSE3 pose = MsgToPose( msg->pose.pose );
		if( !initialized )
		{
			_lastPose = pose;
			_lastPoseTime = msg->header.stamp;
			initialized = true;
			return;
		}
		if( msg->header.stamp - _lastPoseTime < _minDt ) { return; }

		PoseSE3 delta = _lastPose.Inverse() * pose;
		double dt = ( msg->header.stamp - _lastPoseTime ).toSec();
		PoseSE3::TangentVector twist = PoseSE3::Log( delta ) / dt;

		geometry_msgs::TwistStamped out;
		out.header = msg->header;
		out.twist = TangentToMsg( twist );
		_twistPub.publish( out );

		_lastPose = pose;
		_lastPoseTime = msg->header.stamp;
	}

};

int main( int argc, char** argv )
{
	ros::init( argc, argv, "odometry_differentiator" );
	ros::NodeHandle nh, ph( "~" );
	OdometryDifferentiator differ( nh, ph );
	ros::spin();
	return 0;
}
