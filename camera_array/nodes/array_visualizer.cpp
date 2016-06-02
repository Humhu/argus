#include <ros/ros.h>

#include "visualization_msgs/Marker.h"
#include "extrinsics_array/ExtrinsicsInfoManager.h"
#include "lookup/LookupInterface.h"
#include "camera_array/CameraArrayStatus.h"
#include "camera_array/CameraStatus.h"
#include "argus_utils/geometry/GeometryUtils.h"
#include "argus_utils/utils/ParamUtils.h"
#include <boost/foreach.hpp>

using namespace argus;

class Visualizer
{
public:

Visualizer( const ros::NodeHandle& nh, const ros::NodeHandle& ph )
: nodeHandle( nh ), privHandle( ph ), extrinsicsManager( lookupInterface ),
xSize( 0.04 ), ySize( 0.04 ), zSize( 0.04 )
{
	std::string lookupNamespace;
	ph.param<std::string>( "lookup_namespace", lookupNamespace, "/lookup" );
	lookupInterface.SetLookupNamespace( lookupNamespace );
	
	unsigned int bufferSize;
	GetParamDefault<unsigned int>( privHandle, "buffer_size", bufferSize, 50 );
	visPub = nodeHandle.advertise<visualization_msgs::Marker>( "status_markers", bufferSize );
	statusSub = nodeHandle.subscribe( "array_status", 
	                                  1,
	                                  &Visualizer::StatusCallback,
	                                  this );
}

void StatusCallback( const camera_array::CameraArrayStatus::ConstPtr& smsg )
{
	BOOST_FOREACH( const camera_array::CameraStatus& status, smsg->status )
	{
		if( !extrinsicsManager.CheckMemberInfo( status.name ) )
		{
			ROS_WARN_STREAM( "Could not find extrinsics for: " << status.name );
			continue;
		}
		
		const ExtrinsicsInfo& info = extrinsicsManager.GetInfo( status.name );
		
		visualization_msgs::Marker msg;
		msg.header.stamp = smsg->header.stamp;
		msg.header.frame_id = info.referenceFrame;
		msg.ns = status.name + "_status";
		msg.type = visualization_msgs::Marker::CYLINDER;
		msg.action = visualization_msgs::Marker::ADD;
		msg.scale.x = xSize;
		msg.scale.y = ySize;
		msg.scale.z = zSize;
		msg.color.a = 1.0;
		msg.id = 0;
		static PoseSE3 offset( 0, 0, 0, 1, 0, 1, 0 );
		msg.pose = PoseToMsg( info.extrinsics * offset );
		
		if( status.status == "active" )
		{
			msg.color.r = 0.0;
			msg.color.g = 1.0;
			msg.color.b = 0.0;
		}
		else if( status.status == "activating" )
		{
			msg.color.r = 0.3;
			msg.color.g = 0.6;
			msg.color.b = 0.3;
		}
		else if( status.status == "inactive" )
		{
			msg.color.r = 0.2;
			msg.color.g = 0.2;
			msg.color.b = 0.2;
		}
		else if( status.status == "deactivating" )
		{
			msg.color.r = 0.6;
			msg.color.g = 0.3;
			msg.color.b = 0.6;
		}
		else
		{
			msg.color.r = 1.0;
			msg.color.g = 0.0;
			msg.color.b = 0.0;
		}
		visPub.publish( msg );
	}
}

private:
	
	ros::NodeHandle nodeHandle;
	ros::NodeHandle privHandle;
	
	ros::Subscriber statusSub;
	ros::Publisher visPub;
	
	ExtrinsicsInfoManager extrinsicsManager;
	LookupInterface lookupInterface;
	
	double xSize;
	double ySize;
	double zSize;

};

int main( int argc, char** argv )
{
	
	ros::init( argc, argv, "array_visualizer" );
	
	ros::NodeHandle nh, ph( "~" );
	Visualizer vis( nh, ph );
	
	ros::spin();
	return 0;
}
