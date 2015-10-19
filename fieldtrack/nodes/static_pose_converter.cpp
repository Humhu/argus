#include <ros/ros.h>

#include "geometry_msgs/PoseStamped.h"
#include "argus_msgs/RelativePose.h"
#include "argus_utils/GeometryUtils.h"

using namespace argus_msgs;
using namespace argus_utils;

void PoseCallback( const RelativePose::ConstPtr& msg,
                   std::string refName,
                   ros::Publisher& pub )
{
	PoseSE3 pose;
	std::string frameName;
	if( msg->observer_header.frame_id == refName )
	{
		pose = MsgToPose( msg->relative_pose );
		frameName = msg->target_header.frame_id;
	}
	else if( msg->target_header.frame_id == refName )
	{
		pose = MsgToPose( msg->relative_pose ).Inverse();
		frameName = msg->observer_header.frame_id;
	}
	else
	{
		return;
	}
	
	geometry_msgs::PoseStamped pmsg;
	pmsg.header.stamp = msg->observer_header.stamp;
	pmsg.header.frame_id = frameName;
	pmsg.pose = PoseToMsg( pose );
	pub.publish( pmsg );
}

int main( int argc, char** argv )
{

	ros::init( argc, argv, "static_pose_converter" );
	
	ros::NodeHandle nh, ph( "~" );
	
	std::string referenceName;
	if( !ph.getParam( "reference_name", referenceName ) )
	{
		ROS_ERROR_STREAM( "Please specify reference frame name." );
		return -1;
	}
	
	ros::Publisher posePub = nh.advertise<geometry_msgs::PoseStamped>
	    ( "poses", 10 );
	
	boost::function< void( const RelativePose::ConstPtr& ) > f =
		boost::bind( &PoseCallback, _1, referenceName,
					 boost::ref( posePub ) );
		
	ros::Subscriber relPoseSub = nh.subscribe( "relative_poses", 10, f );
	
	ros::spin();
	
	return 0;
}
