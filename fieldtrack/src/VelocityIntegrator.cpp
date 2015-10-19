#include "fieldtrack/VelocityIntegrator.h"
#include "argus_utils/GeometryUtils.h"
#include "argus_utils/YamlUtils.h"

using namespace argus_utils;

namespace fieldtrack
{
	
VelocityIntegrator::VelocityIntegrator( ros::NodeHandle& nh, ros::NodeHandle& ph )
: nodeHandle( nh ), privHandle( ph ), initialized( false )
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
	msg.observer_header.stamp = event.last_real;
	msg.observer_header.frame_id = referenceName;
	msg.target_header.stamp = event.current_real;
	msg.target_header.frame_id = referenceName;
	msg.relative_pose = PoseToMsg( integratedPose );
	dispPub.publish( msg );
	
	integratedPose = PoseSE3();
}

void VelocityIntegrator::TwistCallback( const geometry_msgs::TwistStamped::ConstPtr& msg )
{
	PoseSE3::TangentVector vel = MsgToTangent( msg->twist );
	integratedPose = integratedPose * offset * PoseSE3::Exp( vel );
}
	
} // end namespace fieldtrack
