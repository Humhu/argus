#include "fieldtrack/BodyFrameTracker.h"
#include "argus_utils/MatrixUtils.h"
#include "argus_utils/GeometryUtils.h"

#include <boost/foreach.hpp>

using namespace argus_msgs;
using namespace argus_utils;

namespace fieldtrack
{

BodyFrameTracker::BodyFrameTracker( const ros::NodeHandle& nh, const ros::NodeHandle& ph )
: nodeHandle( nh ), privHandle( ph ), targetManager( lookupInterface )
{
	if( !privHandle.getParam( "reference_frame", referenceFrame ) )
	{
		ROS_ERROR_STREAM( "Must specify reference frame." );
		exit( -1 );
	}
	
	std::string lookupNamespace;
	privHandle.param<std::string>( "lookup_namespace", lookupNamespace, "/lookup" );
	lookupInterface.SetLookupNamespace( lookupNamespace );
	
	targetPub = nodeHandle.advertise<CompactOdometryArray>( "target_states", 5 );
	refOdomSub = nodeHandle.subscribe( "reference_odometry",
	                                   10,
	                                   &BodyFrameTracker::OdometryCallback,
	                                   this );
	targetPoseSub = nodeHandle.subscribe( "target_pose", 
	                                      10,
	                                      &BodyFrameTracker::TargetPoseCallback,
	                                      this );
	
	double updateRate;
	privHandle.param<double>( "update_rate", updateRate, 10.0 );
	updateTimer = std::make_shared<ros::Timer> 
		( nodeHandle.createTimer( ros::Duration( 1.0/updateRate ),
		                          &BodyFrameTracker::TimerCallback,
		                          this ) );
}

void BodyFrameTracker::TimerCallback( const ros::TimerEvent& event )
{
	if( !lastOdometry )
	{
		// ROS_WARN_STREAM( "No odometry received yet." );
		return;
	}
	double dt = ( event.current_real - event.last_real ).toSec();
	ReadLock lock( mutex );
	PoseSE3::TangentVector velocity = MsgToTangent( lastOdometry->twist.twist );
	PoseSE3::CovarianceMatrix velocityCov;
	ParseMatrix( lastOdometry->twist.covariance, velocityCov );
	lock.unlock();
	PoseSE3 displacement = PoseSE3::Exp( dt * velocity );
	PoseSE3::CovarianceMatrix displacementCov = velocityCov * dt;
	
	// Publish target states
	CompactOdometryArray::Ptr msg = boost::make_shared<CompactOdometryArray>();
	msg->header.stamp = event.current_real;
	msg->header.frame_id = referenceFrame;
	msg->odometry.reserve( targetRegistry.size() );
	
	BOOST_FOREACH( const TargetRegistry::value_type& item, targetRegistry )
	{
		const std::string& name = item.first;
		const TargetRegistration& registration = item.second;
		if( !registration.poseInitialized ) { continue; }
		
		// Update target pose
		registration.filter->DisplaceReference( displacement, displacementCov );
		registration.filter->Predict( event.current_real );
		
		// Publish state 
		CompactOdometry odom;
		odom.header.stamp = event.current_real;
		odom.header.frame_id = referenceFrame;
		odom.child_frame_id = name;
		odom.pose.pose = PoseToMsg( registration.filter->PoseFilter().EstimateMean() );
		SerializeSymmetricMatrix( registration.filter->PoseFilter().EstimateCovariance(),
		                          odom.pose.covariance );
		odom.twist.twist = TangentToMsg( registration.filter->VelocityFilter().EstimateMean() );
		SerializeSymmetricMatrix( registration.filter->VelocityFilter().EstimateCovariance(),
		                          odom.twist.covariance );
		
		msg->odometry.push_back( odom );
	}
	
	targetPub.publish( msg );
}

void BodyFrameTracker::OdometryCallback( const nav_msgs::Odometry::ConstPtr& msg )
{
	WriteLock lock( mutex ); // TODO Technically not necessary since single-threaded
	lastOdometry = msg;
}

void BodyFrameTracker::TargetPoseCallback( const RelativePoseWithCovariance::ConstPtr& msg )
{
	if( msg->relative_pose.observer_name != referenceFrame ) 
	{ 
		ROS_WARN_STREAM( "Received relative pose with observer " << msg->relative_pose.observer_name 
		                 << " instead of reference frame " << referenceFrame );
		return; 
	}
		
	const std::string& name = msg->relative_pose.target_name;
	if( targetRegistry.count( name ) == 0 ) { RegisterTarget( name ); }
	
	TargetRegistration& registration = targetRegistry[ name ];
	PoseSE3 pose = MsgToPose( msg->relative_pose.relative_pose );
	PoseSE3::CovarianceMatrix poseCov;
	ParseSymmetricMatrix( msg->covariance, poseCov );
	
	if( registration.poseInitialized )
	{
		registration.filter->PoseUpdate( pose, poseCov, msg->header.stamp );
	}
	else
	{
		registration.filter->PoseFilter().EstimateMean() = pose;
		registration.filter->PoseFilter().EstimateCovariance() = poseCov;
		registration.poseInitialized = true;
	}
}

void BodyFrameTracker::RegisterTarget( const std::string& name )
{
	if( targetRegistry.count( name ) > 0 ) { return; }
	
	// TODO Look up target information to set motion model
	TargetRegistration registration;
	registration.filter = std::make_shared<ConstantVelocityFilter>();
	registration.poseInitialized = false;
	targetRegistry[ name ] = registration;
}
	
}
