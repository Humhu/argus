#include <ros/ros.h>

#include "camplex/DriverNode.h"
#include "argus_utils/utils/ParamUtils.h"

using namespace argus;

int main( int argc, char** argv )
{
	ros::init( argc, argv, "camera_driver" );
	
	ros::NodeHandle nh;
	ros::NodeHandle ph( "~" );
	
	DriverNode node( nh, ph );
	ros::spin();
	
	return 0;
}
