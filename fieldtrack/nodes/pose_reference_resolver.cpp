#include <ros/ros.h>

#include "argus_msgs/RelativePose.h"
#include "argus_utils/GeometryUtils.h"
#include "lookup/LookupInterface.h"
#include "extrinsics_array/ExtrinsicsInfoManager.h"

using namespace argus_msgs;
using namespace argus_utils;

class PoseReferenceResolver 
{
public:

	PoseReferenceResolver( const ros::NodeHandle& nh, const ros::NodeHandle& ph )
	: nodeHandle( nh ), privHandle( ph ), lookupInterface(),
	extrinsicsManager( lookupInterface )
	{
		if( !privHandle.getParam( "reference_name", referenceName ) )
		{
			ROS_ERROR_STREAM( "Please specify reference frame name." );
			exit( -1 );
		}
		
		std::string lookupNamespace;
		privHandle.param<std::string>( "lookup_namespace", lookupNamespace, "/lookup" );
		lookupInterface.SetLookupNamespace( lookupNamespace );
		
		poseSub = nodeHandle.subscribe( "relative_poses", 10, 
		                                &PoseReferenceResolver::PoseCallback, 
                                        this );
		posePub = nodeHandle.advertise<argus_msgs::RelativePose>( "poses", 10 );
	}

private:
	
	ros::NodeHandle nodeHandle;
	ros::NodeHandle privHandle;
	
	lookup::LookupInterface lookupInterface;
	extrinsics_array::ExtrinsicsInfoManager extrinsicsManager;
	
	ros::Publisher posePub;
	ros::Subscriber poseSub;
	std::string referenceName;
	
	bool CheckName( const std::string& name )
	{
		if( extrinsicsManager.HasMember( name ) ) { return true; }
		return extrinsicsManager.ReadMemberInformation( name, false );
	}
	
	void ProcessInversionAndSend( RelativePose& msg )
	{
		RelativePose out = msg;
		
		if( out.target_name == referenceName )
		{
			std::string temp = out.observer_name;
			out.observer_name = referenceName;
			out.target_name = temp;
			
			out.relative_pose = PoseToMsg( MsgToPose( out.relative_pose ).Inverse() );
		}
		
		posePub.publish( out );
	}
	
	void PoseCallback( const RelativePose::ConstPtr& msg )
	{
		RelativePose msgCopy( *msg );
		std::string observerName = msgCopy.observer_name;
		std::string targetName = msgCopy.target_name;
		
		// If the message already contains the reference, process it and send it out
		if( observerName == referenceName || targetName == referenceName )
		{
			ProcessInversionAndSend( msgCopy );
			return;
		}
		
		PoseSE3 relative = MsgToPose( msg->relative_pose );
		PoseSE3 transformed;
		if( CheckName( observerName ) )
		{
			std::string observerReference = extrinsicsManager.GetReferenceFrame( observerName );
			if( observerReference == referenceName )
			{
				const PoseSE3& observerExtrinsics = extrinsicsManager.GetExtrinsics( observerName );
				transformed = observerExtrinsics * relative;
				msgCopy.observer_name = observerReference;
			}
		}
		
		if( CheckName( targetName ) )
		{
			std::string targetReference = extrinsicsManager.GetReferenceFrame( targetName );
			if( targetReference == referenceName )
			{
				const PoseSE3& targetExtrinsics = extrinsicsManager.GetExtrinsics( targetName );
				transformed = relative * targetExtrinsics.Inverse();
				msgCopy.target_name = targetReference;
			}
		}
		
		msgCopy.relative_pose = PoseToMsg( transformed );
		ProcessInversionAndSend( msgCopy );
	}
};

int main( int argc, char** argv )
{

	ros::init( argc, argv, "static_pose_converter" );
	
	ros::NodeHandle nh, ph( "~" );
	PoseReferenceResolver converter( nh, ph );
	
	ros::spin();
	
	return 0;
}
