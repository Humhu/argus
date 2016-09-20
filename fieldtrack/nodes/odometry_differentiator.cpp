#include <ros/ros.h>

#include "nav_msgs/Odometry.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/PoseStamped.h"

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

		GetParamRequired( ph, "output_mode", _outputMode );
		if( _outputMode == "twist_stamped" )
		{
			_outputPub = nh.advertise<geometry_msgs::TwistStamped>( "output", buffLen );
		}
		else if( _outputMode == "odometry" )
		{
			_outputPub = nh.advertise<nav_msgs::Odometry>( "output", buffLen );
		}
		else
		{
			throw std::invalid_argument( "Unknown output mode: " + _outputMode );
		}

		std::string inputMode;
		GetParamRequired( ph, "input_mode", inputMode );
		if( inputMode == "odometry" )
		{
			_inputSub = nh.subscribe( "input", 
			                         buffLen, 
			                         &OdometryDifferentiator::OdomCallback, 
			                         this );
		}
		else if( inputMode == "pose_stamped" )
		{
			_inputSub = nh.subscribe( "input",
			                          buffLen,
			                          &OdometryDifferentiator::PoseStampedCallback,
			                          this );
		}
		else
		{
			throw std::invalid_argument( "Unknown input mode: " + inputMode );
		}
	}

private:

	ros::Publisher _outputPub;
	ros::Subscriber _inputSub;

	bool initialized;
	ros::Duration _minDt;
	std::string _outputMode;

	PoseSE3 _lastPose;
	ros::Time _lastPoseTime;

	void OdomCallback( const nav_msgs::Odometry::ConstPtr& msg )
	{
		PoseSE3 pose = MsgToPose( msg->pose.pose );
		ProcessPose( pose, msg->header );
	}

	void PoseStampedCallback( const geometry_msgs::PoseStamped::ConstPtr& msg )
	{
		PoseSE3 pose = MsgToPose( msg->pose );
		ProcessPose( pose, msg->header );
	}

	void ProcessPose( const PoseSE3& pose, 
	                  const std_msgs::Header& header )
	{
		if( !initialized )
		{
			_lastPose = pose;
			_lastPoseTime = header.stamp;
			initialized = true;
			return;
		}
		if( header.stamp - _lastPoseTime < _minDt ) { return; }

		PoseSE3 delta = _lastPose.Inverse() * pose;
		double dt = ( header.stamp - _lastPoseTime ).toSec();
		PoseSE3::TangentVector twist = PoseSE3::Log( delta ) / dt;

		if( _outputMode == "twist_stamped" )
		{
			geometry_msgs::TwistStamped out;
			out.header = header;
			out.twist = TangentToMsg( twist );
			_outputPub.publish( out );
		}
		else if( _outputMode == "odometry" )
		{
			nav_msgs::Odometry out;
			out.header = header;
			out.pose.pose = PoseToMsg( pose );
			out.twist.twist = TangentToMsg( twist );
			_outputPub.publish( out );
		}

		_lastPose = pose;
		_lastPoseTime = header.stamp;
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
