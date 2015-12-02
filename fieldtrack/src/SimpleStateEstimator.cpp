#include "fieldtrack/SimpleStateEstimator.h"
#include "argus_utils/GeometryUtils.h"
#include "argus_utils/MatrixUtils.h"

using namespace argus_msgs;
using namespace geometry_msgs;
using namespace nav_msgs;
using namespace argus_utils;

namespace fieldtrack
{

SimpleStateEstimator::SimpleStateEstimator( const ros::NodeHandle& nh, 
                                            const ros::NodeHandle& ph )
: nodeHandle( nh ), privHandle( ph )
{
	if( !privHandle.getParam( "reference_frame", referenceFrame ) )
	{
		ROS_ERROR_STREAM( "Must specify world reference frame." );
		exit( -1 );
	}
	if( !privHandle.getParam( "body_frame", bodyFrame ) )
	{
		ROS_ERROR_STREAM( "Must specify body reference frame." );
		exit( -1 );
	}
	
	// Initialize velocity filter
	velocityFilter.TransMatrix() = VelocityFilter::StateTransition::Identity();
	velocityFilter.ObsMatrix() = VelocityFilter::ObservationMatrix::Identity();
	velocityFilter.EstimateMean() = VelocityFilter::StateVector::Zero();
	velocityFilter.EstimateCovariance() = VelocityFilter::StateCovariance::Identity();
	lastVelocityUpdate = ros::Time::now();
	
	std::vector<double> velocityCovVals;
	if( privHandle.getParam( "velocity_covariance_rate", velocityCovVals ) )
	{
		if( !ParseMatrix( velocityCovVals, velocityCovarianceRate ) )
		{
			ROS_ERROR_STREAM( "Could not parse velocity covariance rate." );
			exit( -1 );
		}
	}
	else
	{
		velocityCovarianceRate = VelocityFilter::StateCovariance::Identity();
	}
	
	// Initialize pose filter
	poseFilter.EstimateMean() = PoseSE3();
	poseFilter.EstimateCovariance() = PoseFilter::StateCovariance::Identity();
	lastPoseUpdate = ros::Time::now();
	
	std::vector<double> poseCovVals;
	if( privHandle.getParam( "pose_covariance_rate", poseCovVals ) )
	{
		if( !ParseMatrix( poseCovVals, poseCovarianceRate ) )
		{
			ROS_ERROR_STREAM( "Could not parse pose covariance rate." );
			exit( -1 );
		}
	}
	else
	{
		poseCovarianceRate = PoseFilter::StateCovariance::Identity();
	}
	
	velSub = nodeHandle.subscribe( "velocity", 
	                               10, 
	                               &SimpleStateEstimator::VelocityCallback,
	                               this );
	poseSub = nodeHandle.subscribe( "relative_pose", 
	                                10, 
	                                &SimpleStateEstimator::PoseCallback,
	                                this );
	odomPub = nodeHandle.advertise<Odometry>( "odometry", 10 );
	
	double timerRate;
	privHandle.param<double>( "timer_rate", timerRate, 10.0 );
	updateTimer = std::make_shared<ros::Timer>(
		nodeHandle.createTimer( ros::Duration( 1.0 / timerRate ),
		                        &SimpleStateEstimator::TimerCallback,
		                        this ) );
}
	
void SimpleStateEstimator::VelocityCallback( const TwistWithCovarianceStamped::ConstPtr& msg )
{
	if( msg->header.frame_id != bodyFrame ) { return; }
	
	double dt = ( msg->header.stamp - lastVelocityUpdate ).toSec();
	if( dt < 0 )
	{
		ROS_WARN_STREAM( "Received velocity message from before last update." );
		return;
	}
	lastVelocityUpdate = msg->header.stamp;
	
	// Velocity update
	velocityFilter.Predict( velocityCovarianceRate * dt );
	
	VelocityFilter::ObservationVector z = MsgToTangent( msg->twist.twist );
	PoseSE3::CovarianceMatrix cov;
	ParseMatrix( msg->twist.covariance, cov );
	velocityFilter.Update( z, cov );
	
}

// TODO Check last pose update time
void SimpleStateEstimator::PoseCallback( const RelativePoseWithCovariance::ConstPtr& msg )
{
	PoseSE3 pose;
	if( msg->relative_pose.observer_name == referenceFrame && 
	    msg->relative_pose.target_name == bodyFrame )
	{
		pose = MsgToPose( msg->relative_pose.relative_pose );
	}
	else if( msg->relative_pose.observer_name == bodyFrame && 
	         msg->relative_pose.target_name == referenceFrame )
	{
		pose = MsgToPose( msg->relative_pose.relative_pose ).Inverse();
	}
	else { 
		ROS_WARN_STREAM( "Observer " << msg->relative_pose.observer_name 
		    << " and target " << msg->relative_pose.target_name 
		    << " do not match reference " << referenceFrame 
		    << " and body " << bodyFrame );
		return; 
	}
	
	PoseSE3::CovarianceMatrix cov;
	ParseMatrix( msg->covariance, cov );
	poseFilter.UpdateBody( pose, cov, BodyFrame );
}

void SimpleStateEstimator::TimerCallback( const ros::TimerEvent& event )
{
	// Use velocity to estimate displacement
	double poseDt = ( event.current_real - lastPoseUpdate ).toSec();
	if( poseDt < 0 )
	{
		ROS_WARN_STREAM( "Estimates are ahead of update timer." );
		return;
	}
	lastPoseUpdate = event.current_real;
	
	// Forward-predict pose with velocity
	PoseSE3 displacement = PoseSE3::Exp( poseDt * velocityFilter.EstimateMean() );
	poseFilter.PredictBody( displacement, 
	                        ( velocityFilter.EstimateCovariance() + poseCovarianceRate ) * poseDt, 
	                        BodyFrame );
	
	Odometry msg;
	msg.header.frame_id = referenceFrame;
	msg.header.stamp = event.current_real;
	msg.child_frame_id = bodyFrame;
	
	msg.pose.pose = PoseToMsg( poseFilter.EstimateMean() );
	SerializeMatrix( poseFilter.EstimateCovariance(), msg.pose.covariance );
	
	msg.twist.twist = TangentToMsg( velocityFilter.EstimateMean() );
	SerializeMatrix( velocityFilter.EstimateCovariance(), msg.twist.covariance );
	
	odomPub.publish( msg );
}

}
