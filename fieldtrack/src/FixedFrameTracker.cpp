#include "fieldtrack/FixedFrameTracker.h"

#include "argus_utils/utils/MatrixUtils.h"
#include "argus_utils/utils/ParamUtils.h"
#include "argus_utils/utils/YamlUtils.h"

#include "argus_msgs/CompactOdometryArray.h"

#include <boost/foreach.hpp>

using namespace argus_msgs;
using namespace geometry_msgs;

namespace argus
{

typedef ConstantVelocityFilterSE3 CVF3;

FixedFrameTracker::FixedFrameTracker( ros::NodeHandle& nh, ros::NodeHandle& ph )
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
	
	double pubFreq;
	ph.param<double>( "update_rate", pubFreq, 10.0 );
	timer = std::make_shared<ros::Timer>
		( nodeHandle.createTimer( ros::Duration( 1.0/pubFreq ), 
		                          &FixedFrameTracker::TimerCallback, 
		                          this ) );
	ROS_INFO_STREAM( "Updating and publishing at rate " << pubFreq );
	
	targetPub = nodeHandle.advertise<CompactOdometryArray>( "target_state", 10 );
	targetPoseSub = nodeHandle.subscribe( "target_pose", 
	                                      10, 
	                                      &FixedFrameTracker::TargetPoseCallback, 
	                                      this );
	targetVelSub = nodeHandle.subscribe( "target_velocity", 
	                                     10, 
	                                     &FixedFrameTracker::TargetVelocityCallback, 
	                                     this );
}

void FixedFrameTracker::TimerCallback( const ros::TimerEvent& event )
{
	CompactOdometryArray::Ptr msg = boost::make_shared<CompactOdometryArray>();
	
	BOOST_FOREACH( TargetRegistry::value_type& item, targetRegistry )
	{
		CompactOdometry odom;
		odom.header.frame_id = referenceFrame;
		odom.header.stamp = event.current_real;
		odom.child_frame_id = item.first;
		
		TargetRegistration& registration = item.second;

		if( event.current_real > registration.filterTime )
		{
			double dt = (event.current_real - registration.filterTime).toSec();
			registration.filter.Predict( registration.Qrate * dt, dt );
			registration.filterTime = event.current_real;
		}
		
		odom.pose.pose = PoseToMsg( registration.filter.Pose() );
		SerializeSymmetricMatrix( registration.filter.PoseCov(), odom.pose.covariance );
		
		PoseSE3::TangentVector tangents = registration.filter.Derivs().head<6>();
		odom.twist.twist = TangentToMsg( tangents );
		SerializeSymmetricMatrix( registration.filter.DerivsCov().topLeftCorner<6,6>(), odom.twist.covariance );
		
		msg->odometry.push_back( odom );
	}
	
	msg->header.frame_id = referenceFrame;
	msg->header.stamp = event.current_real;
	targetPub.publish( msg );
}

void FixedFrameTracker::TargetPoseCallback( const RelativePoseWithCovariance::ConstPtr& msg )
{
	
	if( msg->relative_pose.observer_name != referenceFrame )
	{
		ROS_WARN_STREAM( "Received relative pose message with observer " << msg->relative_pose.observer_name
			<< " unrelated to reference " << referenceFrame );
		return;
	}
	
	std::string targetName = msg->relative_pose.target_name;
	if( targetRegistry.count( targetName ) == 0 ) { RegisterTarget( targetName ); }
	
	TargetRegistration& registration = targetRegistry[ targetName ];
	PoseSE3 pose = MsgToPose( msg->relative_pose.relative_pose ); // Target pose relative to observer
	PoseSE3::CovarianceMatrix relCov;
	ParseSymmetricMatrix( msg->covariance, relCov );
	
	if( registration.poseInitialized )
	{
		if( msg->header.stamp > registration.filterTime )
		{
			double dt = (msg->header.stamp - registration.filterTime).toSec();
			registration.filter.Predict( registration.Qrate * dt, dt );
			registration.filterTime = msg->header.stamp;
		}
		registration.filter.UpdatePose( pose, relCov );
	}
	else
	{
		registration.filter.Pose() = pose;
		registration.filter.PoseCov() = relCov;
		registration.poseInitialized = true;
		registration.filterTime = msg->header.stamp;
	}
	
}

void FixedFrameTracker::TargetVelocityCallback( const TwistWithCovarianceStamped::ConstPtr& msg )
{
	const std::string& targetName = msg->header.frame_id;
	
	if( targetRegistry.count( targetName ) == 0 ) { RegisterTarget( targetName ); }
	TargetRegistration& registration = targetRegistry[ targetName ];
	
	if( msg->header.stamp > registration.filterTime )
	{
		double dt = (msg->header.stamp - registration.filterTime).toSec();
		registration.filter.Predict( registration.Qrate * dt, dt );
		registration.filterTime = msg->header.stamp;
	}

	PoseSE3::TangentVector velocity = MsgToTangent( msg->twist.twist );
	PoseSE3::CovarianceMatrix velocityCov;
	ParseMatrix( msg->twist.covariance, velocityCov );
	registration.filter.UpdateDerivs( velocity, CVF3::DerivObsMatrix::Identity(6,6),
	                                  velocityCov );
	
}

void FixedFrameTracker::RegisterTarget( const std::string& name )
{
	if( targetRegistry.count( name ) > 0 ) { return; }
	
	ROS_INFO_STREAM( "Registering target " << name << " for tracking." );
	
	targetRegistry[ name ];
	
}

FixedFrameTracker::TargetRegistration::TargetRegistration()
: poseInitialized( false ), filterTime( ros::Time::now() ),
filter( PoseSE3(), CVF3::DerivsType::Zero(), 10*CVF3::FullCovType::Identity() ),
Qrate( CVF3::FullCovType::Identity() )
{}

} // end namespace fieldtrack
