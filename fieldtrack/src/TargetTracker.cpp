#include "fieldtrack/TargetTracker.h"
#include "geometry_msgs/PoseStamped.h" // TODO Publish covariance also!

#include "argus_utils/YamlUtils.h"

#include <boost/foreach.hpp>

using namespace argus_utils;
using namespace argus_msgs;

namespace fieldtrack
{
	
TargetTracker::TargetTracker( ros::NodeHandle& nh, ros::NodeHandle& ph )
: nodeHandle( nh ), privHandle( ph ), firstIteration ( true )
{
	YAML::Node iCov, tCov, oCov;
	if( !GetYamlParam( privHandle, "initial_covariance", iCov ) ||
	    !GetYamlParam( privHandle, "transition_covariance_rate", tCov ) ||
	    !GetYamlParam( privHandle, "observation_covariance", oCov ) )
	{
		ROS_ERROR_STREAM( "Please specify tracker covariances." );
		exit( -1 );
	}
	
	Eigen::MatrixXd iQ, tQ, oQ;
	if( !GetMatrixYaml( iCov, iQ ) || !GetMatrixYaml( tCov, tQ ) || 
	    !GetMatrixYaml( oCov, oQ ) )
	{
		ROS_ERROR_STREAM( "Could not parse covariance YAML." );
		exit( -1 );
	}
	
	if( iQ.rows() != initCovariance.rows() || iQ.cols() != initCovariance.cols() ||
		tQ.rows() != transCovariance.rows() || tQ.cols() != transCovariance.cols() ||
		oQ.rows() != obsCovariance.rows() || oQ.cols() != obsCovariance.cols() )
	{
		ROS_ERROR_STREAM( "Parsed matrices are of the wrong size." );
	}
	
	initCovariance = iQ;
	transCovariance = tQ;
	obsCovariance = oQ;
	
	if( !privHandle.getParam( "reference_name", referenceName ) )
	{
		ROS_ERROR_STREAM( "Reference name not specified." );
		exit( -1 );
	}
	ROS_INFO_STREAM( "Tracking targets relative to " << referenceName );
	
	double pubFreq;
	ph.param<double>( "update_rate", pubFreq, 10.0 );
	timer = std::make_shared<ros::Timer>( 
	    nodeHandle.createTimer( ros::Duration( 1.0/pubFreq ), &TargetTracker::TimerCallback, this ) );
	ROS_INFO_STREAM( "Updating and publishing at rate " << pubFreq );
	
	poseSub = nodeHandle.subscribe( "relative_poses", 10, &TargetTracker::PoseCallback, this );
}

// TODO Odometry update from displacement updates

void TargetTracker::TimerCallback( const ros::TimerEvent& event )
{
	// Don't predict on first iteration because dt is not defined
	firstIteration = false;
	
	double dt = ( event.current_real - event.last_real ).toSec();
	
	// TODO Predict step to current time?
	typedef std::unordered_map< std::string, Filter > FilterMap;
	BOOST_FOREACH( FilterMap::value_type& item, filters )
	{
		std::string targetName = item.first;
		Filter& filter = item.second;
		if( !firstIteration )
		{
			filter.Predict( transCovariance * dt, WorldFrame );
		}
		
		argus_msgs::RelativePose msg;
		msg.observer_header.frame_id = referenceName;
		msg.observer_header.stamp = event.current_real;
		msg.target_header.frame_id = targetName;
		msg.target_header.stamp = event.current_real;
		msg.relative_pose = PoseToMsg( filter.EstimateMean() );
		publishers[ targetName ].publish( msg );
	}
}

void TargetTracker::PoseCallback( const RelativePose::ConstPtr& msg )
{
	PoseSE3 relPose;
	std::string targetName;
	
	if( msg->observer_header.frame_id == referenceName )
	{
		relPose = MsgToPose( msg->relative_pose );
		targetName = msg->target_header.frame_id;
	}
	else if( msg->target_header.frame_id == referenceName )
	{
		relPose = MsgToPose( msg->relative_pose ).Inverse();
		targetName = msg->observer_header.frame_id;
	}
	else
	{
		ROS_WARN_STREAM( "Received relative pose message unrelated to frame " << referenceName );
		return;
	}
	
	// If there isn't already a filter, create it
	if( filters.count( targetName ) == 0 )
	{
		ROS_INFO_STREAM( "Observed new target " << targetName );
		
		filters[ targetName ] = Filter();
		Filter& filter = filters[ targetName ];
		filter.EstimateMean() = relPose;
		filter.EstimateCovariance() = initCovariance;
		filter.TransCovariance() = transCovariance;
		filter.ObsCovariance() = obsCovariance;
		
		publishers[ targetName ] = nodeHandle.advertise<argus_msgs::RelativePose>
		    ( "tracked_frames", 10 );
	}
	
// 	ros::Duration age = ros::Time::now() - msg->observer_header.stamp;
// 	ROS_INFO_STREAM( "Observation age: " << age.toSec() );
	
	// Retrieve the filter and perform an update
	Filter& filter = filters[ targetName ];
	filter.UpdateWorld( relPose, BodyFrame ); // TODO Incorporate extrinsics uncertainty
	
}
	
} // end namespace fieldtrack
