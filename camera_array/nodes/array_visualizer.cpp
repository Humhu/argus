#include <ros/ros.h>

#include "visualization_msgs/Marker.h"
#include "extrinsics_array/ExtrinsicsInfoManager.h"
#include "lookup/LookupInterface.h"
#include "camera_array/CameraArrayStatus.h"
#include "camera_array/CameraStatus.h"
#include "argus_utils/GeometryUtils.h"
#include <boost/foreach.hpp>

using namespace camera_array;
using namespace argus_utils;

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
	
	visPub = nodeHandle.advertise<visualization_msgs::Marker>( "status_markers", 1 );
	statusSub = nodeHandle.subscribe( "array_status", 
	                                  1,
	                                  &Visualizer::StatusCallback,
	                                  this );
}

void StatusCallback( const CameraArrayStatus::ConstPtr& smsg )
{
	BOOST_FOREACH( const CameraStatus& status, smsg->status )
	{
		if( !extrinsicsManager.CheckMemberInfo( status.name ) )
		{
			ROS_WARN_STREAM( "Could not find extrinsics for: " << status.name );
			continue;
		}
		
		const extrinsics_array::ExtrinsicsInfo& info = extrinsicsManager.GetInfo( status.name );
		
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
		msg.pose = PoseToMsg( info.extrinsics );
		
		if( status.status == "active" )
		{
			msg.color.r = 0.0;
			msg.color.g = 1.0;
			msg.color.b = 0.0;
		}
		else if( status.status == "activating" )
		{
			msg.color.r = 0.3;
			msg.color.g = 0.8;
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
			msg.color.r = 0.1;
			msg.color.g = 0.8;
			msg.color.b = 0.1;
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
	
	extrinsics_array::ExtrinsicsInfoManager extrinsicsManager;
	lookup::LookupInterface lookupInterface;
	
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
