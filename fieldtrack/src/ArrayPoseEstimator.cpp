#include "fieldtrack/ArrayPoseEstimator.h"
#include "argus_utils/GeometryUtils.h"

using namespace argus_utils;
using namespace extrinsics_array;

namespace fieldtrack
{
	
ArrayPoseEstimator::ArrayPoseEstimator( ros::NodeHandle& nh, ros::NodeHandle& ph )
: nodeHandle( nh ), privHandle( ph ), extrinsicsManager( nh )
{
	// Set up lookup interface
	std::string lookupNamespace;
	if( !ph.getParam( "lookup_namespace", lookupNamespace ) ) 
	{
		ROS_ERROR_STREAM( "Lookup namespace must be specified." );
	}
	if( lookupNamespace.back() != '/' ) { lookupNamespace += "/"; }
	extrinsicsManager.SetLookupNamespace( lookupNamespace );
	
	// Parse reference name
	if( !privHandle.getParam( "reference_name", referenceName ) )
	{
		ROS_ERROR_STREAM( "Reference name not specified." );
		exit( -1 );
	}
	ROS_INFO_STREAM( "Tracking targets relative to " << referenceName );
	
	// Subscribe to relative pose observations
	relPoseSub = nodeHandle.subscribe( "input_poses", 10, 
	                                   &ArrayPoseEstimator::RelativePoseCallback, this );
}

void ArrayPoseEstimator::RelativePoseCallback( const argus_msgs::RelativePose::ConstPtr& msg )
{
	PoseSE3 obsPose;
	std::string targetName;
	
	// Flip reversed observations if needed
	if( msg->observer_name == referenceName )
	{
		obsPose = MsgToPose( msg->relative_pose );
		targetName = msg->target_name;
	}
	else if( msg->target_name == referenceName )
	{
		obsPose = MsgToPose( msg->relative_pose ).Inverse();
		targetName = msg->observer_name;
	}
	else
	{
		ROS_WARN_STREAM( "Received relative pose message unrelated to frame " << referenceName );
		return;
	}
	
	// Query array information about the viewed frame
	if( !extrinsicsManager.HasMember( targetName ) )
	{
		// We allow cached lookup here so that we don't keep querying the master
		// for an untrackable object
		if( !extrinsicsManager.ReadMemberInformation( targetName, false ) )
		{
// 			ROS_WARN_STREAM( "Could not retrieve extrinsics info for " << targetName );
			return;
		}
	}
	
	// Convert obsPose to be between the frames referenceName and target's parent array frame
	const ExtrinsicsArray& targetArray = extrinsicsManager.GetParentArray( targetName );
	std::string targetFrameName = targetArray.GetReferenceFrame();
	PoseSE3 targetExtrinsics = targetArray.GetPose( targetName );
	
	PoseSE3 relPose = obsPose * targetExtrinsics.Inverse();
	
	// Create publisher if needed
	if( relPosePubs.count( targetFrameName ) == 0 )
	{
		relPosePubs[ targetFrameName ] = 
		    nodeHandle.advertise<argus_msgs::RelativePose>( "output_poses", 10 );
	}
	
	// Publish the relative pose
	argus_msgs::RelativePose pMsg;
	pMsg.observer_name = referenceName;
	pMsg.observer_time = msg->observer_time;
	pMsg.target_name = targetFrameName;
	pMsg.target_time = msg->target_time;
	pMsg.relative_pose = PoseToMsg( relPose );
	relPosePubs[ targetFrameName ].publish( pMsg );
	
}
	
} // end namespace fieldtrack
