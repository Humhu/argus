#include "fieldtrack/VelocityIntegrator.h"
#include "argus_utils/GeometryUtils.h"
#include "argus_utils/YamlUtils.h"

using namespace argus_utils;

namespace fieldtrack
{
	
VelocityIntegrator::VelocityIntegrator( ros::NodeHandle& nh, ros::NodeHandle& ph )
: nodeHandle( nh ), privHandle( ph ), initialized( false ), twistInitialized( false )
{
	YAML::Node poseYaml;
	if( GetYamlParam( privHandle, "transform", poseYaml ) )
	{
		if( !GetPoseYaml( poseYaml, offset ) )
		{
			ROS_ERROR( "Could not parse offset transform." );
			exit( -1 );
		}
	}
	
	privHandle.param( "scale", scale, 1.0 );
	
	if( !privHandle.getParam( "reference_name", referenceName ) )
	{
		ROS_ERROR( "Reference name not specified." );
		exit( -1 );
	}
	
	dispPub = nodeHandle.advertise<argus_msgs::RelativePose>( "displacements", 10 );
	twistSub = nodeHandle.subscribe( "body_velocities", 10, 
	                                 &VelocityIntegrator::TwistCallback, this );
	
	double updateRate;
	privHandle.param( "update_rate", updateRate, 10.0 );
	timer = std::make_shared<ros::Timer>
	    ( nodeHandle.createTimer( ros::Duration( 1.0/updateRate ),
	                              &VelocityIntegrator::TimerCallback,
	                              this ) );
}

void VelocityIntegrator::TimerCallback( const ros::TimerEvent& event )
{
	if( !initialized )
	{
		initialized = true;
		return;
	}
	
	argus_msgs::RelativePose msg;
	msg.observer_time = event.last_real;
	msg.observer_name = referenceName;
	msg.target_time = event.current_real;
	msg.target_name = referenceName;
	msg.relative_pose = PoseToMsg( integratedPose );
	dispPub.publish( msg );
	
	integratedPose = PoseSE3();
}

void VelocityIntegrator::TwistCallback( const geometry_msgs::TwistStamped::ConstPtr& msg )
{
	if( twistInitialized )
	{
		PoseSE3::TangentVector currVel = MsgToTangent( msg->twist );
		PoseSE3::TangentVector prevVel = MsgToTangent( lastTwist.twist );
		PoseSE3::TangentVector meanVel = 0.5 * ( currVel + prevVel );
		
		double dt = ( msg->header.stamp - lastTwist.header.stamp ).toSec();
		PoseSE3 displacement = 
		    PoseSE3::Exp( dt * scale * PoseSE3::Adjoint( offset ) * meanVel ).Inverse();
		integratedPose = integratedPose * displacement;
	}
	twistInitialized = true;
	lastTwist = *msg;
}
	
} // end namespace fieldtrack
