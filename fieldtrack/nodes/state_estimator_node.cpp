#include <ros/ros.h>
#include "fieldtrack/SimpleStateEstimator.h"
#include "argus_utils/utils/ParamUtils.h"

using namespace argus;

int main( int argc, char** argv )
{
	ros::init( argc, argv, "state_estimator_node" );
	
	ros::NodeHandle nh;
	ros::NodeHandle ph( "~" );
	argus::SimpleStateEstimator estimator( nh, ph );

	unsigned int numSpinners;
	GetParam( ph, "num_threads", numSpinners, (unsigned int) 1 );

	//ros::spin();
	ros::AsyncSpinner spinner( numSpinners );
	spinner.start();
	ros::waitForShutdown();

	return 0;
}
