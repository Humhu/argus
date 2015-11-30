#include "fieldtrack/SimpleStateEstimator.h"
#include "argus_utils/GeometryUtils.h"
#include "argus_utils/MatrixUtils.h"

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
	
	dispSub = nodeHandle.subscribe( "displacements", 
	                                10, 
	                                &SimpleStateEstimator::DisplacementCallback,
	                                this );
	poseSub = nodeHandle.subscribe( "poses", 
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
	
void SimpleStateEstimator::DisplacementCallback( const PoseWithCovarianceStamped::ConstPtr& msg )
{
	if( msg->header.frame_id != bodyFrame ) { return; }
	
	double dt = ( msg->header.stamp - lastVelocityUpdate ).toSec();
	if( dt < 0 )
	{
		ROS_WARN_STREAM( "Received displacement message from before last update." );
		return;
	}
	
	// Velocity update
	velocityFilter.Predict( velocityCovarianceRate * dt );
	
	PoseSE3 displacement = MsgToPose( msg->pose.pose );
	VelocityFilter::ObservationVector z = PoseSE3::Log( displacement );
	PoseSE3::CovarianceMatrix cov;
	ParseMatrix( msg->pose.covariance, cov );
	velocityFilter.Update( z, cov );
	
	lastVelocityUpdate = msg->header.stamp;
	
	// Pose displacement
	poseFilter.PredictBody( displacement, cov + poseCovarianceRate * dt, BodyFrame );
	lastPoseUpdate = msg->header.stamp;
}

void SimpleStateEstimator::PoseCallback( const PoseWithCovarianceStamped::ConstPtr& msg )
{
	if( msg->header.frame_id != referenceFrame ) { return; }
	
	PoseSE3 pose = MsgToPose( msg->pose.pose );
	PoseSE3::CovarianceMatrix cov;
	ParseMatrix( msg->pose.covariance, cov );
	poseFilter.UpdateBody( pose, cov, BodyFrame );
}

void SimpleStateEstimator::TimerCallback( const ros::TimerEvent& event )
{
	// TODO This might be slow...
	// Initialize copies for estimating
	VelocityFilter velocityEstimator( velocityFilter );
	PoseFilter poseEstimator( poseFilter );
	ros::Time velEstimateTime = lastVelocityUpdate;
	ros::Time poseEstimateTime = lastPoseUpdate;
	
	// Use velocity to estimate displacement
	double poseDt = ( event.current_real - poseEstimateTime ).toSec();
	double velDt = ( event.current_real - velEstimateTime ).toSec();
	if( poseDt < 0 || velDt < 0 )
	{
		ROS_WARN_STREAM( "Estimates are ahead of update timer." );
		return;
	}
	
	// Forward-predict pose with velocity
	PoseSE3 displacement = PoseSE3::Exp( poseDt * velocityEstimator.EstimateMean() );
	PoseSE3::CovarianceMatrix displacementCov = poseDt * velocityEstimator.EstimateCovariance();
	poseEstimator.PredictBody( displacement, 
	                           displacementCov + poseCovarianceRate * poseDt, 
	                           BodyFrame );
	
	// Predict velocity
	velocityEstimator.Predict( velocityCovarianceRate * velDt );
	
	Odometry msg;
	msg.header.frame_id = referenceFrame;
	msg.header.stamp = event.current_real;
	msg.child_frame_id = bodyFrame;
	
	msg.pose.pose = PoseToMsg( poseEstimator.EstimateMean() );
	SerializeMatrix( poseEstimator.EstimateCovariance(), msg.pose.covariance );
	
	msg.twist.twist = TangentToMsg( velocityEstimator.EstimateMean() );
	SerializeMatrix( velocityEstimator.EstimateCovariance(), msg.twist.covariance );
	
	odomPub.publish( msg );
}

}
