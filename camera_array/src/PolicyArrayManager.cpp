#include "camera_array/PolicyArrayManager.h"

using namespace argus_utils;

namespace camera_array
{
	
PolicyArrayManager::PolicyArrayManager( const ros::NodeHandle& nh,
                                        const ros::NodeHandle& ph )
: CameraArrayManager( nh, ph )
{
	odomSub = nodeHandle.subscribe( "odometry", 5, &PolicyArrayManager::OdometryCallback, this );
	
	double updateRate;
	privHandle.param<double>( "update_rate", updateRate, 1.0 );
	updateTimer = std::make_shared<ros::Timer>
		( nodeHandle.createTimer( ros::Duration( 1.0/updateRate ),
		                          &PolicyArrayManager::TimerCallback,
		                          this ) );
}

void PolicyArrayManager::TimerCallback( const ros::TimerEvent& event )
{
	if( !lastOdometry ) { return; }
	double dt = ( event.current_real - lastOdometry->header.stamp ).toSec();
	
	PoseSE3 lastPose = MsgToPose( lastOdometry->pose.pose );
	PoseSE3::CovarianceMatrix lastPoseCov;
	ParseMatrix( lastOdometry->pose.covariance, lastPoseCov );
	PoseSE3::TangentVector lastVelocity = MsgToTangent( lastOdometry->twist.twist );
	PoseSE3::CovarianceMatrix lastVelocityCov;
	ParseMatrix( lastOdometry->twist.covariance, lastVelocityCov );
	
	PoseSE3 estimatedPose = lastPose * PoseSE3::Exp( dt * lastVelocity );
	PoseSE3::CovarianceMatrix estimatedCov = lastPoseCov + dt*lastVelocityCov;
	
	
}

void PolicyArrayManager::OdometryCallback( const nav_msgs::Odometry::ConstPtr& msg )
{
	if( msg->header.frame_id != referenceFrame ||
	    msg->child_frame_id != bodyFrame )
	{
		ROS_ERROR_STREAM( "Odometry frames do not match reference/body frames." );
		exit( -1 );
	}
	lastOdometry = msg;
}

}
