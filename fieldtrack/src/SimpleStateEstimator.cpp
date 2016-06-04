#include "fieldtrack/SimpleStateEstimator.h"
#include "argus_utils/geometry/GeometryUtils.h"
#include "argus_utils/utils/MatrixUtils.h"
#include "argus_utils/utils/ParamUtils.h"
#include "argus_utils/filters/FilterUtils.h"

using namespace argus_msgs;
using namespace geometry_msgs;
using namespace nav_msgs;


namespace argus
{

SimpleStateEstimator::SimpleStateEstimator( const ros::NodeHandle& nh, 
                                            const ros::NodeHandle& ph )
: nodeHandle( nh ), privHandle( ph ), 
filter( PoseSE3(), FilterType::DerivsType::Zero(), FilterType::FullCovType::Zero() )
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
	
	Qrate = FilterType::FullCovType::Zero();
	FixedMatrixType<6,6> Qvel;
	if( !GetMatrixParam<double>( privHandle, "velocity_covariance_rate", Qvel ) )
	{
		ROS_WARN_STREAM( "No velocity covariance rate given. Using identity." );
		Qvel = FixedMatrixType<6,6>::Identity();
	}
	Qrate.block<6,6>(6,6) = Qvel;
	Qrate.block<6,6>(12,12) = Qvel;

	FilterType::PoseCovType Qpos;
	if( !GetMatrixParam<double>( privHandle, "pose_covariance_rate", Qpos ) )
	{
		ROS_WARN_STREAM( "No pose covariance rate given. Using identity." );
		Qpos = FilterType::PoseCovType::Identity();
	}
	Qrate.topLeftCorner<FilterType::TangentDim,FilterType::TangentDim>() = Qpos;

	filter.Pose() = PoseSE3();
	filter.Derivs() = FilterType::DerivsType::Zero();
	filter.FullCov() = 10 * FilterType::FullCovType::Identity();
	filterTime = ros::Time::now();

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
	stepPub = nodeHandle.advertise<argus_msgs::FilterStepInfo>( "filter_info", 10 );
	
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
	PoseSE3::CovarianceMatrix R;
	ParseMatrix( msg->twist.covariance, R );
	
	ros::Time now = msg->header.stamp;
	if( now > filterTime )
	{
		double dt = (now - filterTime).toSec();
		PredictInfo info = filter.Predict( Qrate*dt, dt );
		argus_msgs::FilterStepInfo pmsg = PredictToMsg( info );
		pmsg.header.stamp = msg->header.stamp;
		stepPub.publish( pmsg );
		filterTime = now;
	}

	FilterType::DerivObsMatrix C = FilterType::DerivObsMatrix::Zero( 6, FilterType::DerivsDim );
	C.leftCols<6>() = FixedMatrixType<6,6>::Identity(6,6);
	UpdateInfo info = filter.UpdateDerivs( velocity, C, R );
	argus_msgs::FilterStepInfo umsg = UpdateToMsg( info );
	umsg.header.stamp = msg->header.stamp;
	stepPub.publish( umsg );
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
	
	ros::Time now = msg->header.stamp;
	if( now > filterTime )
	{
		double dt = (now - filterTime).toSec();
		PredictInfo info = filter.Predict( Qrate*dt, dt );
		argus_msgs::FilterStepInfo pmsg = PredictToMsg( info );
		pmsg.header.stamp = msg->header.stamp;
		stepPub.publish( pmsg );
		filterTime = now;
	}

	PoseSE3::CovarianceMatrix cov;
	ParseSymmetricMatrix( msg->covariance, cov );
	UpdateInfo info = filter.UpdatePose( pose, cov );
	argus_msgs::FilterStepInfo umsg = UpdateToMsg( info );
	stepPub.publish( umsg );
}

void SimpleStateEstimator::TimerCallback( const ros::TimerEvent& event )
{
	ros::Time now = event.current_real;
	if( now > filterTime )
	{
		double dt = (now - filterTime).toSec();
		PredictInfo info = filter.Predict( Qrate*dt, dt );
		argus_msgs::FilterStepInfo pmsg = PredictToMsg( info );
		pmsg.header.stamp = event.current_real;
		stepPub.publish( pmsg );
		filterTime = now;
	}
	if( twoDimensional ) { EnforceTwoDimensionality(); }

	Odometry msg;
	msg.header.frame_id = referenceFrame;
	msg.header.stamp = event.current_real;
	msg.child_frame_id = bodyFrame;
	
	msg.pose.pose = PoseToMsg( filter.Pose() );
	SerializeMatrix( filter.PoseCov(), msg.pose.covariance );
	
	msg.twist.twist = TangentToMsg( filter.Derivs().head<6>() );
	SerializeMatrix( filter.DerivsCov().topLeftCorner<6,6>(), msg.twist.covariance );
	
	odomPub.publish( msg );
}

void SimpleStateEstimator::EnforceTwoDimensionality()
{
  FixedVectorType<7> poseVector = filter.Pose().ToVector();
  PoseSE3 mean( poseVector(0), poseVector(1), 0, poseVector(3), 0, 0, poseVector(6) );
  filter.Pose() = mean;

  PoseSE3::TangentVector velocity = filter.Derivs().head<6>();
  velocity(2) = 0;
  velocity(3) = 0;
  velocity(4) = 0;
  filter.Derivs().head<6>() = velocity;
}

}
