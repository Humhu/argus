#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <argus_utils/utils/ParamUtils.h>
#include <unordered_map>

using namespace argus;

class TransformRepeater
{
public:

	TransformRepeater( ros::NodeHandle& nh, ros::NodeHandle& ph ) 
	{
		unsigned int buffSize;
		GetParam( ph, "buffer_size", buffSize, (unsigned int) 10 );
		_sub = nh.subscribe( "transform", 
		                     buffSize,
		                     &TransformRepeater::TransformCallback,
		                     this );
	}

private:

	ros::Subscriber _sub;
	tf2_ros::TransformBroadcaster _broadcaster;

	void TransformCallback( const geometry_msgs::TransformStamped::ConstPtr& msg )
	{
		_broadcaster.sendTransform( *msg );
	}
};

int main( int argc, char** argv )
{
	ros::init( argc, argv, "transform_repeater_node" );
	ros::NodeHandle nh, ph( "~" );
	TransformRepeater tr( nh, ph );
	ros::spin();

	return 0;
}