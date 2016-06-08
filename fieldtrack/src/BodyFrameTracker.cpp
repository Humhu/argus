#include "fieldtrack/BodyFrameTracker.h"
#include "argus_utils/utils/MatrixUtils.h"
#include "argus_utils/geometry/GeometryUtils.h"

#include <boost/foreach.hpp>

using namespace argus_msgs;

namespace argus
{

typedef ConstantVelocityFilterSE3 CVF3;

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
	PoseSE3 displacement;
	PoseSE3::CovarianceMatrix displacementCov;
	if( lastOdometry )
	{
		double dt = ( event.current_real - event.last_real ).toSec();
		ReadLock lock( mutex );
		PoseSE3::TangentVector velocity = MsgToTangent( lastOdometry->twist.twist );
		PoseSE3::CovarianceMatrix velocityCov;
		ParseMatrix( lastOdometry->twist.covariance, velocityCov );
		lock.unlock();
		displacement = PoseSE3::Exp( dt * velocity );
		displacementCov = velocityCov * dt;
	}

	// Publish target states
	CompactOdometryArray::Ptr msg = boost::make_shared<CompactOdometryArray>();
	msg->header.stamp = event.current_real;
	msg->header.frame_id = referenceFrame;
	msg->odometry.reserve( targetRegistry.size() );
	
	BOOST_FOREACH( TargetRegistry::value_type& item, targetRegistry )
	{
		const std::string& name = item.first;
		TargetRegistration& registration = item.second;
		if( !registration.poseInitialized ) { continue; }
		
		// Update target pose
		if( lastOdometry )
		{
			// TODO This logic might be broken
			double dt = (event.current_real - registration.filterTime).toSec();
			registration.filter.Predict( registration.Qrate*dt, dt );
			registration.filter.WorldDisplace( displacement.Inverse(), displacementCov );
			registration.filterTime = event.current_real;
		}

		// Publish state 
		CompactOdometry odom;
		odom.header.stamp = event.current_real;
		odom.header.frame_id = referenceFrame;
		odom.child_frame_id = name;
		odom.pose.pose = PoseToMsg( registration.filter.Pose() );
		SerializeSymmetricMatrix( registration.filter.PoseCov(),
		                          odom.pose.covariance );
		PoseSE3::TangentVector tangents = registration.filter.Derivs().head<6>();
		odom.twist.twist = TangentToMsg( tangents );
		SerializeSymmetricMatrix( registration.filter.DerivsCov().topLeftCorner<6,6>(),
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
		if( msg->header.stamp > registration.filterTime )
		{
			double dt = (msg->header.stamp - registration.filterTime).toSec();
			registration.filter.Predict( registration.Qrate * dt, dt );
			registration.filterTime = msg->header.stamp;
		}
		registration.filter.UpdatePose( pose, poseCov );
	}
	else
	{
		registration.filter.Pose() = pose;
		registration.filter.Derivs() = CVF3::DerivsType::Zero();
		registration.filter.FullCov() = 10 * CVF3::FullCovType::Identity();
		registration.filter.FullCov().topLeftCorner<CVF3::TangentDim,CVF3::TangentDim>()
			= poseCov;
		registration.poseInitialized = true;
		registration.filterTime = msg->header.stamp;
	}
}

void BodyFrameTracker::RegisterTarget( const std::string& name )
{
	if( targetRegistry.count( name ) > 0 ) { return; }

	ROS_INFO_STREAM( "Registering target " << name );

	// TODO Look up target information to set motion model
	targetRegistry[ name ];
}

BodyFrameTracker::TargetRegistration::TargetRegistration()
: poseInitialized( false ), filterTime( ros::Time::now() ),
filter( PoseSE3(), CVF3::DerivsType::Zero(), 10*CVF3::FullCovType::Identity() ),
Qrate( CVF3::FullCovType::Identity() ) {}

}
