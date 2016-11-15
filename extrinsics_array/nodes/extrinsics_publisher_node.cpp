#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <argus_utils/geometry/GeometryUtils.h>
#include <argus_utils/utils/ParamUtils.h>
#include <unordered_map>

using namespace argus;

int main( int argc, char** argv )
{
	ros::init( argc, argv, "extrinsics_publisher_node" );
	ros::NodeHandle nh, ph( "~" );
	tf2_ros::StaticTransformBroadcaster tfBroadcaster;

	geometry_msgs::TransformStamped msg;
	msg.header.stamp = ros::Time::now();

	YAML::Node transforms;
	GetParamRequired( ph, "", transforms );
	YAML::Node::const_iterator iter;
	for( iter = transforms.begin(); iter != transforms.end(); ++iter )
	{
		// TODO For some reason this segfaults if it's a const &
		YAML::Node info = iter->second;

		GetParamRequired( info, "parent_id", msg.header.frame_id );
		msg.child_frame_id = iter->first.as<std::string>();
		PoseSE3 pose;
		GetParamRequired( info, "pose", pose );
		msg.transform = PoseToTransform( pose );
		
		ROS_INFO_STREAM( "Publishing extrinsics for: " << msg.child_frame_id << 
		                 " relative to " << msg.header.frame_id <<
		                 " of " << pose );
		tfBroadcaster.sendTransform( msg );
	}

	ros::spin();

	return 0;
}