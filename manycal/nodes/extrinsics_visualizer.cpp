#include <ros/ros.h>

#include "argus_utils/PoseSE3.h"
#include "argus_utils/GeometryUtils.h"
#include "argus_utils/ParamUtils.h"
#include "argus_utils/YamlUtils.h"
#include "visualization_msgs/Marker.h"

#include "lookup/LookupInterface.h"

using namespace argus;

class Visualizer 
{
public:
	
	Visualizer( const ros::NodeHandle& nh, const ros::NodeHandle& ph )
	: nodeHandle( nh ), privHandle( ph )
	{
		if( !privHandle.getParam( "target_frame", frameName ) )
		{
			ROS_ERROR_STREAM( "Must specify frame to visualize." );
			exit( -1 );
		}
		ROS_INFO_STREAM( "Crawling extrinsics for " << frameName );
		
		std::string lookupNamespace;
		privHandle.param<std::string>( "lookup_namespace", lookupNamespace, "/lookup" );
		lookupInterface.SetLookupNamespace( lookupNamespace );
		
		visPub = nodeHandle.advertise<visualization_msgs::Marker>( "markers", 10 );
		
		privHandle.param( "refresh_rate", refreshRate, 1.0 );
		
		timer = std::make_shared<ros::Timer>( 
		    nodeHandle.createTimer( ros::Duration( 1.0/refreshRate ),
			                        boost::bind( &Visualizer::Visualize, this, _1 ) ) );
	}
	
	// TODO Clear markers after?
	
	void Visualize( const ros::TimerEvent& event )
	{
		std::string targetNamespace;
		if( !lookupInterface.ReadNamespace( frameName, targetNamespace ) )
		{
			ROS_ERROR_STREAM( "Could not read namespace for target: " << frameName );
			exit( -1 );
		}
		YAML::Node dump;
		if( !GetYamlParam( nodeHandle, targetNamespace, dump ) )
		{
			ROS_ERROR_STREAM( "Could not retrieve parameters from: " << targetNamespace );
			exit( -1 );
		}
		
		ProcessNode( dump );
	}
		
	void ProcessNode( const YAML::Node& node )
	{
		if( !node.IsMap() ) { return; }
		
		YAML::Node::const_iterator iter;
		for( iter = node.begin(); iter != node.end(); iter++ )
		{
			const std::string& name = iter->first.as<std::string>();
			const YAML::Node& child = iter->second;
			if( !child.IsMap() ) { continue; }
			if( !child[ "extrinsics" ] ) 
			{ 
				ProcessNode( child ); 
				continue;
			}
			
			PoseSE3 pose;
			if( !GetPoseYaml( child[ "extrinsics" ], pose ) )
			{
				ROS_WARN_STREAM( "Could not parse extrinsics for: " << name );
				continue;
			}
			
			static PoseSE3 yTrans( 0, 0, 0, 1, 0, 0, 1 );
			static PoseSE3 zTrans( 0, 0, 0, 1, 0, -1, 0 );
			
			ROS_INFO_STREAM( "Visualizing extrinsics for " << name );
			visualization_msgs::Marker msg;
			msg.header.stamp = ros::Time::now();
			msg.header.frame_id = frameName;
			msg.ns = name;
			msg.type = visualization_msgs::Marker::ARROW;
			msg.action = visualization_msgs::Marker::ADD;
			msg.scale.x = 0.5;
			msg.scale.y = 0.01;
			msg.scale.z = 0.01;
			msg.color.a = 1.0;
			msg.lifetime = ros::Duration( 1.0/refreshRate );
			
			// x-axis
			msg.id = 0;
			msg.pose = PoseToMsg( pose );
			msg.color.r = 1.0;
			msg.color.g = 0.0;
			msg.color.b = 0.0;
			visPub.publish( msg );
			
			// y-axis
			msg.id = 1;
			msg.pose = PoseToMsg( pose*yTrans );
			msg.color.r = 0.0;
			msg.color.g = 1.0;
			msg.color.b = 0.0;
			visPub.publish( msg );
			
			// z-axis
			msg.id = 2;
			msg.pose = PoseToMsg( pose*zTrans );
			msg.color.r = 0.0;
			msg.color.g = 0.0;
			msg.color.b = 1.0;
			visPub.publish( msg );
			
			// TODO Set scale parameter for node
			msg.id = 3;
			msg.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
			msg.pose = PoseToMsg( pose );
			msg.text = name;
			msg.scale.x = 0.5;
			msg.scale.y = 0.5;
			msg.scale.z = 0.5;
			msg.color.r = 1.0;
			msg.color.g = 1.0;
			msg.color.b = 1.0;
			visPub.publish( msg );
			
		}
	}

private:
	
	ros::NodeHandle nodeHandle;
	ros::NodeHandle privHandle;
	
	lookup::LookupInterface lookupInterface;
	std::shared_ptr<ros::Timer> timer;
	
	double refreshRate;
	std::string frameName;
	ros::Publisher visPub;
};

int main( int argc, char** argv )
{
	ros::init( argc, argv, "extrinsics_visualizer" );
	
	ros::NodeHandle nh, ph( "~" );
	Visualizer vis( nh, ph );
	
	ros::spin();
	return 0;
}
