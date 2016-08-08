#include "paraset/DiscretePolicyManager.h"
#include <ros/ros.h>

using namespace argus;

int main( int argc, char** argv )
{
	ros::init( argc, argv, "discrete_policy_node" );

	ros::NodeHandle nh, ph( "~" );
	DiscretePolicyManager policy;
	policy.Initialize( nh, ph );
	ros::spin();
	return 0;
}