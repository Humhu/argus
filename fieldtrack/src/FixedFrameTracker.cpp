#include "fieldtrack/FixedFrameTracker.h"

#include "argus_utils/MatrixUtils.h"
#include "argus_utils/ParamUtils.h"
#include "argus_utils/YamlUtils.h"

#include "argus_msgs/CompactOdometryArray.h"

#include <boost/foreach.hpp>

using namespace argus;
using namespace argus_msgs;
using namespace geometry_msgs;

namespace fieldtrack
{
	
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
	
	BOOST_FOREACH( const TargetRegistry::value_type& item, targetRegistry )
	{
		CompactOdometry odom;
		odom.header.frame_id = referenceFrame;
		odom.header.stamp = event.current_real;
		odom.child_frame_id = item.first;
		
		const TargetRegistration& registration = item.second;
		registration.filter->Predict( event.current_real );
		
		const ConstantVelocityFilter::PoseFilterType& poseFilter = 
			registration.filter->PoseFilter();
		odom.pose.pose = PoseToMsg( poseFilter.EstimateMean() );
		SerializeSymmetricMatrix( poseFilter.EstimateCovariance(), odom.pose.covariance );
		
		const ConstantVelocityFilter::VelocityFilterType& velFilter = 
			registration.filter->VelocityFilter();
		odom.twist.twist = TangentToMsg( velFilter.EstimateMean() );
		SerializeSymmetricMatrix( velFilter.EstimateCovariance(), odom.twist.covariance );
		
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
		registration.filter->PoseUpdate( pose, relCov, msg->header.stamp );
	}
	else
	{
		registration.filter->PoseFilter().EstimateMean() = pose;
		registration.filter->PoseFilter().EstimateCovariance() = relCov;
		registration.poseInitialized = true;
	}
	
}

void FixedFrameTracker::TargetVelocityCallback( const TwistWithCovarianceStamped::ConstPtr& msg )
{
	const std::string& targetName = msg->header.frame_id;
	
	if( targetRegistry.count( targetName ) == 0 ) { RegisterTarget( targetName ); }
	TargetRegistration& registration = targetRegistry[ targetName ];
	
	PoseSE3::TangentVector velocity = MsgToTangent( msg->twist.twist );
	PoseSE3::CovarianceMatrix velocityCov;
	ParseMatrix( msg->twist.covariance, velocityCov );
	registration.filter->VelocityUpdate( velocity, velocityCov, msg->header.stamp );
	
}

void FixedFrameTracker::RegisterTarget( const std::string& name )
{
	if( targetRegistry.count( name ) > 0 ) { return; }
	
	ROS_INFO_STREAM( "Registering target " << name << " for tracking." );
	
	TargetRegistration registration;
	registration.poseInitialized = false;
	registration.filter = std::make_shared<ConstantVelocityFilter>();
	targetRegistry[ name ] = registration;
	
}

} // end namespace fieldtrack
