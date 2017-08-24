#include <ros/ros.h>

#include "camplex/SplitStereoDriverNode.h"
#include "argus_utils/utils/ParamUtils.h"

using namespace argus;

int main( int argc, char** argv )
{
	ros::init( argc, argv, "split_camera_driver" );
	
	ros::NodeHandle nh;
	ros::NodeHandle ph( "~" );
	
	SplitStereoDriverNode node( nh, ph );
	ros::spin();
	
	return 0;
}
