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
		ROS_ERROR_STREAM( "Must specify state reference frame." );
		exit( -1 );
	}
	if( !privHandle.getParam( "body_frame", bodyFrame ) )
	{
		ROS_ERROR_STREAM( "Must specify body reference frame." );
		exit( -1 );
	}
		
	std::vector<double> velocityCovVals;
	if( privHandle.getParam( "velocity_covariance_rate", velocityCovVals ) )
	{
		if( !ParseMatrix( velocityCovVals, filter.VelocityCovarianceRate() ) )
		{
			ROS_ERROR_STREAM( "Could not parse velocity covariance rate." );
			exit( -1 );
		}
	}
	else
	{
		filter.VelocityCovarianceRate() = PoseSE3::CovarianceMatrix::Identity();
	}
	
	// Initialize pose filter
	std::vector<double> poseCovVals;
	if( privHandle.getParam( "pose_covariance_rate", poseCovVals ) )
	{
		if( !ParseMatrix( poseCovVals, filter.PoseCovarianceRate() ) )
		{
			ROS_ERROR_STREAM( "Could not parse pose covariance rate." );
			exit( -1 );
		}
	}
	else
	{
		filter.PoseCovarianceRate() = PoseSE3::CovarianceMatrix::Identity();
	}

	privHandle.param<bool>( "two_dimensional", twoDimensional, false );
	
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
	
	PoseSE3::TangentVector velocity = MsgToTangent( msg->twist.twist );
	PoseSE3::CovarianceMatrix cov;
	ParseMatrix( msg->twist.covariance, cov );
	
	filter.VelocityUpdate( velocity, cov, msg->header.stamp );
	
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
	ParseSymmetricMatrix( msg->covariance, cov );
	filter.PoseUpdate( pose, cov, msg->header.stamp );
}

void SimpleStateEstimator::TimerCallback( const ros::TimerEvent& event )
{
	filter.Predict( event.current_real );
	if( twoDimensional ) { EnforceTwoDimensionality(); }
	
	Odometry msg;
	msg.header.frame_id = referenceFrame;
	msg.header.stamp = event.current_real;
	msg.child_frame_id = bodyFrame;
	
	ConstantVelocityFilter::PoseFilterType& poseFilter = filter.PoseFilter();
	msg.pose.pose = PoseToMsg( poseFilter.EstimateMean() );
	SerializeMatrix( poseFilter.EstimateCovariance(), msg.pose.covariance );
	
	ConstantVelocityFilter::VelocityFilterType& velocityFilter = filter.VelocityFilter();
	msg.twist.twist = TangentToMsg( velocityFilter.EstimateMean() );
	SerializeMatrix( velocityFilter.EstimateCovariance(), msg.twist.covariance );
	
	odomPub.publish( msg );
}

void SimpleStateEstimator::EnforceTwoDimensionality()
{
  PoseSE3::Vector poseVector = filter.PoseFilter().EstimateMean().ToVector();
  PoseSE3 mean( poseVector(0), poseVector(1), 0, poseVector(3), 0, 0, poseVector(6) );
  filter.PoseFilter().EstimateMean() = mean;

  PoseSE3::TangentVector velocity = filter.VelocityFilter().EstimateMean();
  velocity(2) = 0;
  velocity(3) = 0;
  velocity(4) = 0;
  filter.VelocityFilter().EstimateMean() = velocity;
}

}
