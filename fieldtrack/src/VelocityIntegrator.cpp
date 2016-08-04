#include "fieldtrack/VelocityIntegrator.h"
#include "argus_utils/geometry/GeometryUtils.h"
#include "argus_utils/utils/ParamUtils.h"
#include "argus_utils/utils/YamlUtils.h"
#include "argus_utils/utils/MatrixUtils.h"

namespace argus
{
	
VelocityIntegrator::VelocityIntegrator( ros::NodeHandle& nh, ros::NodeHandle& ph )
: nodeHandle( nh ), privHandle( ph ), initialized( false ), twistInitialized( false )
{
	YAML::Node poseYaml;
	if( GetParam( privHandle, "transform", poseYaml ) )
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
	
	geometry_msgs::PoseWithCovarianceStamped msg;
	msg.header.frame_id = referenceName;
	msg.header.stamp = event.current_real;
	msg.pose.pose = PoseToMsg( integratedPose );
	SerializeMatrix( integratedCovariance, msg.pose.covariance );
	dispPub.publish( msg );
	
	integratedPose = PoseSE3();
	integratedCovariance = PoseSE3::CovarianceMatrix::Zero();
}

void VelocityIntegrator::TwistCallback( const geometry_msgs::TwistWithCovarianceStamped::ConstPtr& msg )
{
	if( twistInitialized )
	{
		PoseSE3::TangentVector currVel = MsgToTangent( msg->twist.twist );
		PoseSE3::TangentVector prevVel = MsgToTangent( lastTwist.twist.twist );
		PoseSE3::TangentVector meanVel = 0.5 * ( currVel + prevVel );
		
		double dt = ( msg->header.stamp - lastTwist.header.stamp ).toSec();
		PoseSE3 displacement = 
		    PoseSE3::Exp( dt * PoseSE3::Adjoint( offset ) * meanVel ).Inverse();
		integratedPose = integratedPose * displacement;

		PoseSE3::CovarianceMatrix cov;
		ParseMatrix( msg->twist.covariance, cov );

		PoseSE3::AdjointMatrix adj = PoseSE3::Adjoint( displacement ).inverse();
		integratedCovariance = adj * integratedCovariance * adj.transpose() + cov;
	}
	twistInitialized = true;
	lastTwist = *msg;
}
	
} // end namespace fieldtrack
