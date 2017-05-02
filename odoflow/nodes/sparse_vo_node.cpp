
#include <sensor_msgs/image_encodings.h>


#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "odoflow/VisualOdometryPipeline.h"

using namespace argus;

int main( int argc, char** argv )
{
	ros::init( argc, argv, "visual_odometry_node" );
	
	ros::NodeHandle nh, ph( "~" );
	
	VisualOdometryPipeline vo( nh, ph );

	unsigned int numThreads;
	GetParam<unsigned int>( ph, "num_threads", numThreads, 2 );
	ros::AsyncSpinner spinner( numThreads );
	spinner.start();
	ros::waitForShutdown();
	
	return 0;
}
