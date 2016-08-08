#include "paraset/ContinuousPolicyManager.h"
#include <ros/ros.h>

using namespace argus;

int main( int argc, char** argv )
{
	ros::init( argc, argv, "continuous_policy_node" );

	ros::NodeHandle ph( "~" );
	ContinuousPolicyManager policy;
	policy.Initialize( ph );
	ros::spin();
	return 0;
}