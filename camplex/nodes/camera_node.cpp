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
	
	unsigned int numThreads;
	GetParam<unsigned int>(ph, "num_threads", numThreads, 1);

	ros::MultiThreadedSpinner spinner( numThreads );
	spinner.spin();

	return 0;
}
